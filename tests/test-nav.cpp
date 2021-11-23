/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-skeleton.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

/*
 * How to launch the test:
 * - load the CER_base_only model within Gazebo 
 * - baseControl --context baseControl_SIM --from baseCtrl_cer_sim.ini --GENERAL::use_ROS false --skip_robot_interface_check
 * - navController 
 * - test-nav 
 * - echo "start test" | yarp rpc /navController/rpc
 */

#include <cstdlib>
#include <memory>
#include <utility>
#include <vector>
#include <cmath>
#include <algorithm>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

class TestNavigator : public RFModule {
  double t0{0.0};
  double robot_x{0.0};
  shared_ptr<Skeleton> skeleton;
  BufferedPort<Bottle> opcPort;
  BufferedPort<Bottle> statePort;

  bool configure(ResourceFinder& rf)override {
    skeleton = shared_ptr<Skeleton>(new SkeletonStd);
    skeleton->setTag("test");
    opcPort.open("/test-nav/opc:rpc");
    statePort.open("/test-nav/state:i");
    if (Network::connect(opcPort.getName(), "/navController/opc:i") &&
        Network::connect("/navController/state:o", statePort.getName())) {
      t0 = Time::now();
      return true;
    } else {
      close();
      return false;
    }
  }

  double getPeriod()override {
    return 0.1;
  }

  bool updateModule()override {
    if (Bottle* b = statePort.read(false)) {
      robot_x = b->get(0).asFloat64();
    }

    Vector pos(3, 0.0);
    double w = (2.0 * M_PI) / 20.0;
    double t = Time::now() - t0;
    pos[2] = 3.0 + 2.0 * sin(w * t);
    pos[2] = max(min(pos[2], 4.0), 2.0) - robot_x;

    vector<pair<string, Vector>> unordered;
    unordered.push_back(make_pair(KeyPointTag::hip_center, pos));
    skeleton->update(unordered);

    // broadcast the skeleton pretending to be the OPC
    Bottle& bottle = opcPort.prepare();
    bottle.clear();
    bottle.addString("sync");

    Bottle& item = bottle.addList();
    item.read(skeleton->toProperty());

    Bottle& idList = item.addList();
    idList.addString("id");
    idList.addInt32(0);

    opcPort.writeStrict();
    return true;
  }

  bool close()override {
    if (!opcPort.isClosed()) {
      opcPort.close();
    }
    if (!statePort.isClosed()) {
      statePort.close();
    }
    return true;
  }
};

int main(int argc, char* argv[]) {
  Network yarp;
  if (!yarp.checkNetwork()) {
    yError() << "Unable to connect to YARP server";
    return EXIT_FAILURE;
  }

  ResourceFinder rf;
  rf.configure(argc, argv);

  TestNavigator test;
  return test.runModule(rf);
}

