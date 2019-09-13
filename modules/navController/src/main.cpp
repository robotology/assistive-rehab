/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <mutex>
#include <memory>
#include <string>
#include <cmath>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include "AssistiveRehab/skeleton.h"
#include "src/navController_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

/****************************************************************/
class Navigator : public RFModule, public navController_IDL {
  double period{0.0}; 
  double velocity_magnitude{0.0};
  double distance_target{0.0}; 
  double distance_hysteresis_low{0.0}; 
  double distance_hysteresis_high{0.0}; 
  bool distance_hysteresis_active{false}; 
  string skeleton_tag{""};
  shared_ptr<Skeleton> skeleton;

  RpcClient navCmdPort;
  BufferedPort<Bottle> navLocPort;
  BufferedPort<Bottle> navCtrlPort;
  BufferedPort<Bottle> opcPort;
  BufferedPort<Bottle> statePort;
  RpcServer cmdPort;

  mutex mtx;

  struct {
    double x{0.0}; 
    double y{0.0}; 
    double theta{0.0}; 
    bool getFrom(BufferedPort<Bottle>& navLocPort) {
      if (Bottle* b = navLocPort.read(false)) {
        x = b->get(0).asDouble();
        y = b->get(1).asDouble();
        theta = b->get(2).asDouble();
        return true;
      }
      return false;
    }
    void print() {
      yInfo() << "Location = (" << x << y << theta << ")";
    }
  } location;

  struct {
    double x{0.0}; 
    double theta{0.0}; 
    void zero() {
      x = theta = 0.0;
    }
    void sendTo(BufferedPort<Bottle>& navCtrlPort) {
      Bottle& b = navCtrlPort.prepare();
      b.clear();
      b.addInt(3);
      b.addDouble(x);
      b.addDouble(0.0);
      b.addDouble(theta);
      b.addDouble(100.0);
      navCtrlPort.writeStrict();
    }
    void print() {
      yInfo() << "Velocity = (" << x << theta << ")";
    }
  } velocity;

  /****************************************************************/
  bool attach(RpcServer& source)override {
    return yarp().attachAsServer(source);
  }

  /****************************************************************/
  bool configure(ResourceFinder& rf)override {
    period = rf.check("period", Value(0.05)).asDouble();
    velocity_magnitude = rf.check("velocity_magnitude", Value(0.3)).asDouble();
    distance_target = rf.check("distance_target", Value(2.0)).asDouble();
    distance_hysteresis_low = rf.check("distance_hysteresis_low", Value(0.2)).asDouble();
    distance_hysteresis_high = rf.check("distance_hysteresis_high", Value(0.3)).asDouble();

    navCmdPort.open("/navController/base/cmd:rpc");
    navLocPort.open("/navController/base/loc:i");
    navCtrlPort.open("/navController/base/ctrl:o");
    opcPort.open("/navController/opc:i");
    statePort.open("/navController/state:o");
    cmdPort.open("/navController/rpc");
    attach(cmdPort);

    if (Network::connect(navCmdPort.getName(), "/baseControl/rpc") &&
        Network::connect("/baseControl/odometry:o", navLocPort.getName()) &&
        Network::connect(navCtrlPort.getName(), "/baseControl/control:i")) {
      Bottle cmd, rep;
      cmd.addString("run");
      if (navCmdPort.write(cmd, rep)) {
        if (rep.size() > 0) {
          cmd.clear();
          cmd.addString("reset_odometry");
          if (navCmdPort.write(cmd, rep)) {
            return (rep.size() > 0);
          }
        }
      }
    }

    yError() << "Unable to talk to baseControl";
    close();
    return false;
  }

  /****************************************************************/
  bool interruptModule()override {
    stop();
    Bottle cmd, rep;
    cmd.addString("idle");
    navCmdPort.write(cmd, rep);
    return true;
  }

  /****************************************************************/
  bool close()override {
    if (navCmdPort.asPort().isOpen())
      navCmdPort.close();
    if (!navLocPort.isClosed())
      navLocPort.close();
    if (!navCtrlPort.isClosed())
      navCtrlPort.close();
    if (!opcPort.isClosed())
      opcPort.close();
    if (!statePort.isClosed())
      statePort.close();
    if (cmdPort.asPort().isOpen())
      cmdPort.close();
    return true;
  }

  /****************************************************************/
  double getPeriod()override {
    return period;
  }

  /****************************************************************/
  void getSkeleton() {
    if (Bottle* b = opcPort.read(false)) {
      if (!b->get(1).isString()) {
        skeleton.reset();
        for (int i = 1; i < b->size(); i++)	{
          Property prop;
          prop.fromString(b->get(i).asList()->toString());
          if (prop.find("tag").asString() == skeleton_tag) {
            skeleton = shared_ptr<Skeleton>(skeleton_factory(prop));
          }
        }
      }
    }
  }

  /****************************************************************/
  void publishState() {
    if (statePort.getOutputCount() > 0) {
      Bottle &b = statePort.prepare();
      b.clear();
      b.addDouble(location.x);
      b.addDouble(location.y);
      b.addDouble(location.theta);
      b.addDouble(velocity.x);
      b.addDouble(velocity.theta);
      statePort.writeStrict();
    }
  }

  /****************************************************************/
  bool updateModule()override {
    lock_guard<mutex> lck(mtx);

    location.getFrom(navLocPort);
    getSkeleton();

    velocity.zero();
    if (skeleton) {
      if ((*skeleton)[KeyPointTag::hip_center]->isUpdated()) {
        double z = (*skeleton)[KeyPointTag::hip_center]->getPoint()[2];
        double e = z - distance_target;
        double abs_e = abs(e);
        double command = velocity_magnitude * sign(e);
        if (distance_hysteresis_active) {
          if (abs_e > distance_hysteresis_high) {
            velocity.x = command;
            distance_hysteresis_active = false;
          }
        } else if (abs_e > distance_hysteresis_low) {
          velocity.x = command;
        } else {
          distance_hysteresis_active = true;
        }
      }
    }
    velocity.sendTo(navCtrlPort);

    publishState();

    location.print();
    velocity.print();

    return true;
  }

  /****************************************************************/
  bool start(const string& skeleton_tag)override {
    lock_guard<mutex> lck(mtx);
    this->skeleton_tag = skeleton_tag;
    yInfo() << "Control started over" << this->skeleton_tag;
    return true;
  }

  /****************************************************************/
  bool stop()override {
    lock_guard<mutex> lck(mtx);
    velocity.zero();
    velocity.sendTo(navCtrlPort);
    skeleton_tag.clear();
    skeleton.reset();
    yInfo() << "Control stopped";
    return true;
  }

  /****************************************************************/
  string which_skeleton()override {
    lock_guard<mutex> lck(mtx);
    return skeleton_tag;
  }
};

/****************************************************************/
int main(int argc, char* argv[]) {
  Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "Unable to connect to YARP server";
    return EXIT_FAILURE;
  }

  ResourceFinder rf;
  rf.configure(argc, argv);

  Navigator navigator;
  return navigator.runModule(rf);
}

