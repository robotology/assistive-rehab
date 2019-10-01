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
#include <chrono>
#include <condition_variable>
#include <memory>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
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
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include "AssistiveRehab/skeleton.h"
#include "src/navController_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace assistive_rehab;

/****************************************************************/
class Navigator : public RFModule, public navController_IDL {
  double period{0.0};
  double velocity_magnitude_linear{0.0};
  double angular_tolerance{0.0};
  double distance_target{0.0};
  double distance_hysteresis_low{0.0};
  double distance_hysteresis_high{0.0};
  bool distance_hysteresis_active{false};
  double target_theta{0.0};
  int heading{1};
  shared_ptr<parallelPID> pid_controller;
  shared_ptr<AWLinEstimator> velocity_estimator;

  shared_ptr<Skeleton> skeleton;
  string skeleton_tag{""};
  Vector skeleton_location{zeros(3)};
  Matrix gaze{zeros(4, 4)};

  RpcClient navCmdPort;
  BufferedPort<Bottle> navLocPort;
  BufferedPort<Bottle> navCtrlPort;
  BufferedPort<Bottle> opcPort;
  BufferedPort<Property> gazePort;
  BufferedPort<Property> statePort;
  RpcServer cmdPort;

  mutex mtx_update;
  mutex mtx_nav_done;
  condition_variable cv_nav_done;

  enum class State {
    idle, track, nav_angular, nav_linear
  } state;

  struct Location {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    Location() = default;
    Location(const double x_, const double y_, const double theta_) : x(x_), y(y_), theta(theta_) { }
    bool getFrom(BufferedPort<Bottle>& navLocPort) {
      if (Bottle* b = navLocPort.read(false)) {
        x = b->get(0).asDouble();
        y = b->get(1).asDouble();
        theta = b->get(2).asDouble();
        return true;
      }
      return false;
    }
  } robot_location;

  vector<Location> target_locations;
  vector<Location>::iterator target_location;

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
  } robot_velocity;

  /****************************************************************/
  bool attach(RpcServer& source)override {
    return yarp().attachAsServer(source);
  }

  /****************************************************************/
  bool configure(ResourceFinder& rf)override {
    period = rf.check("period", Value(0.05)).asDouble();
    velocity_magnitude_linear = abs(rf.check("velocity-magnitude-linear", Value(0.3)).asDouble());
    angular_tolerance = abs(rf.check("angular-tolerance", Value(5.0)).asDouble());
    distance_target = abs(rf.check("distance-target", Value(2.0)).asDouble());
    distance_hysteresis_low = abs(rf.check("distance-hysteresis-low", Value(0.2)).asDouble());
    distance_hysteresis_high = abs(rf.check("distance-hysteresis-high", Value(0.3)).asDouble());

    navCmdPort.open("/navController/base/cmd:rpc");
    navLocPort.open("/navController/base/loc:i");
    navCtrlPort.open("/navController/base/ctrl:o");
    opcPort.open("/navController/opc:i");
    gazePort.open("/navController/gaze:i");
    statePort.open("/navController/state:o");
    cmdPort.open("/navController/rpc");
    attach(cmdPort);

    state = State::idle;
    gaze = numeric_limits<double>::quiet_NaN();

    double Kp = 1.0;
    double Ki = 1.0;
    double Kd = 0.5;
    double w = 1.0;
    double N = 1.0;
    double Tt = 1.0;
    Matrix sat(1, 2);
    sat(0, 0) = -20.0;
    sat(0, 1) = -sat(0, 0);
    pid_controller = shared_ptr<parallelPID>(new parallelPID(period, Kp * ones(1), Ki * ones(1), Kd * ones(1),
                                                             w * ones(1), w * ones(1), w * ones(1),
                                                             N * ones(1), Tt * ones(1), sat));
    velocity_estimator = shared_ptr<AWLinEstimator>(new AWLinEstimator(16, 0.05));

    if (Network::connect(navCmdPort.getName(), "/baseControl/rpc") &&
        Network::connect("/baseControl/odometry:o", navLocPort.getName()) &&
        Network::connect(navCtrlPort.getName(), "/baseControl/control:i")) {
      Bottle cmd, rep;
      cmd.addString("run");
      if (navCmdPort.write(cmd, rep)) {
        if (rep.size() > 0) {
          return reset_odometry();
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
    opcPort.interrupt();
    cv_nav_done.notify_all();
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
    if (!gazePort.isClosed())
      gazePort.close();
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
  void getSkeleton(const bool wait = false) {
    if (opcPort.getInputCount() > 0) {
      if (Bottle* b = opcPort.read(wait)) {
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
  }

  /****************************************************************/
  void getGaze() {
    if (Property* p = gazePort.read(false)) {
      if (Bottle* b = p->find("depth").asList()) {
        Vector pos(3);
        for (size_t i = 0; i < pos.length(); i++) {
          pos[i] = b->get(i).asDouble();
        }
        Vector ax(4);
        for (size_t i = 0; i < ax.length(); i++) {
          ax[i] = b->get(pos.length() + i).asDouble();
        }
        gaze = axis2dcm(ax);
        gaze.setSubcol(pos, 0, 3);
      }
    }
  }

  /****************************************************************/
  Property publishState() {
    Bottle b_rloc, b_rvel, b_tloc, b_sloc;
    Bottle& lb_rloc = b_rloc.addList();
    Bottle& lb_rvel = b_rvel.addList();
    Bottle& lb_tloc = b_tloc.addList();
    Bottle& lb_sloc = b_sloc.addList();

    Property p;
    if (state == State::idle) {
      p.put("robot-state", "idle");
    } else if (state == State::track) {
      p.put("robot-state", "track");
    } else {
      p.put("robot-state", "nav");
    }

    lb_rloc.addDouble(robot_location.x);
    lb_rloc.addDouble(robot_location.y);
    lb_rloc.addDouble(robot_location.theta);
    p.put("robot-location", b_rloc.get(0));

    lb_rvel.addDouble(robot_velocity.x);
    lb_rvel.addDouble(robot_velocity.theta);
    p.put("robot-velocity", b_rvel.get(0));

    if ((state == State::nav_angular) ||
        (state == State::nav_linear)) {
      lb_tloc.addDouble(target_location->x);
      lb_tloc.addDouble(target_location->y);
      lb_tloc.addDouble(target_location->theta);
      lb_tloc.addInt(heading);
      p.put("target-location", b_tloc.get(0));
    }

    if (state == State::track) {
      p.put("skeleton-tag", skeleton_tag);
      Vector v_sloc = skeleton_location;
      v_sloc.push_back(1.0);
      v_sloc = gaze * v_sloc;
      lb_sloc.addDouble(v_sloc[0]);
      lb_sloc.addDouble(v_sloc[1]);
      p.put("skeleton-location", b_sloc.get(0));
    }

    return p;
  }

  /****************************************************************/
  Vector compute_error(const Location& loc) {
    Vector e(2);
    e[0] = loc.x - robot_location.x;
    e[1] = loc.y - robot_location.y;
    return e;
  }

  /****************************************************************/
  void generate_waypoints(const Location& loc) {
    double ang = max(angular_tolerance, 1.0);
    double segment = 0.8 * distance_hysteresis_low / sin(CTRL_DEG2RAD * ang);
    Vector e = compute_error(loc);
    Vector dir = e / norm(e);
    int n = (int)(norm(e) / segment);

    target_locations.clear();
    for (int i = 0; i < n; i++) {
      Location tmp_loc;
      tmp_loc.x = robot_location.x + (i + 1) * segment * dir[0];
      tmp_loc.y = robot_location.y + (i + 1) * segment * dir[1];
      tmp_loc.theta = loc.theta;
      target_locations.push_back(tmp_loc);
    }
    target_locations.push_back(loc);
  }

  /****************************************************************/
  double compute_target_theta() {
    Vector e = compute_error(*target_location);
    double theta = robot_location.theta;
    if (norm(e) > distance_hysteresis_low) {
      theta = CTRL_RAD2DEG * atan2(e[1], e[0]);
      if (heading < 0) {
        theta -= sign(theta) * 180.0;
      }
    }
    return theta;
  }

  /****************************************************************/
  bool updateModule()override {
    lock_guard<mutex> lck(mtx_update);

    robot_location.getFrom(navLocPort);
    getSkeleton();
    getGaze();

    robot_velocity.zero();
    skeleton_location = numeric_limits<double>::quiet_NaN();

    if (state == State::track) {
      if (skeleton) {
        if ((*skeleton)[KeyPointTag::hip_center]->isUpdated()) {
          skeleton_location = (*skeleton)[KeyPointTag::hip_center]->getPoint();
          double e = norm(skeleton_location) - distance_target;
          double abs_e = abs(e);
          double command = velocity_magnitude_linear * sign(e);
          if (distance_hysteresis_active) {
            if (abs_e > distance_hysteresis_high) {
              robot_velocity.x = command;
              distance_hysteresis_active = false;
              yInfo() << "Tracking" << skeleton_tag << ": navigation activated";
            }
          } else if (abs_e > distance_hysteresis_low) {
            robot_velocity.x = command;
          } else {
            distance_hysteresis_active = true;
            yInfo() << "Tracking" << skeleton_tag << ": navigation deactivated";
          }
        }
      }
    } else if (state == State::nav_angular) {
      if (abs(target_theta - robot_location.theta) > angular_tolerance) {
        robot_velocity.theta = pid_controller->compute(target_theta * ones(1), robot_location.theta * ones(1))[0];
      } else {
        Vector e = compute_error(*target_location);
        if (norm(e) > distance_hysteresis_low) {
          velocity_estimator->reset();
          state = State::nav_linear;
          yInfo() << "Starting nav_linear: reaching (" << target_location->x << target_location->y << ") [m]";
        } else {
          target_locations.clear();
          state = State::idle;
          cv_nav_done.notify_all();
          yInfo() << "Target location reached";
        }
      }
    } else if (state == State::nav_linear) {
      Vector e = compute_error(*target_location);
      double norm_e = norm(e);
      AWPolyElement el(Vector(1, norm_e), Time::now());
      double vel_norm_e = velocity_estimator->estimate(el)[0];
      if ((norm_e > distance_hysteresis_low) && (vel_norm_e <= 0.0)) {
        double theta_rad = CTRL_DEG2RAD * robot_location.theta;
        Vector dir(2);
        dir[0] = cos(theta_rad);
        dir[1] = sin(theta_rad);
        robot_velocity.x = sign(dot(dir, e)) * velocity_magnitude_linear;
      } else {
        if (target_location + 1 != target_locations.end()) {
          target_location++;
          target_theta = compute_target_theta();
        } else {
          target_theta = target_location->theta;
        }
        state = State::nav_angular;
        pid_controller->reset(zeros(1));
        yInfo() << "Starting nav_angular: reaching" << target_theta << "[deg]";
      }
    }

    robot_velocity.sendTo(navCtrlPort);

    if (statePort.getOutputCount() > 0) {
      statePort.prepare() = publishState();
      statePort.writeStrict();
    }

    return true;
  }

  /****************************************************************/
  void go_to_helper(const double x, const double y, const double theta,
                    const bool heading_rear) {
    heading = (heading_rear ? -1 : 1);
    yInfo() << "Navigating to (" << x << y << theta << ") heading =" << heading;
    generate_waypoints(Location(x, y, theta));
    target_location = target_locations.begin();
    Vector e = compute_error(*target_location);
    target_theta = ((target_location + 1 == target_locations.end()) && (norm(e) < distance_hysteresis_low) ?
                    target_location->theta : compute_target_theta());
    pid_controller->reset(zeros(1));
    state = State::nav_angular;
    yInfo() << "Starting nav_angular: reaching" << target_theta << "[deg]";
  }

  /****************************************************************/
  bool go_to(const double x, const double y, const double theta,
             const bool heading_rear)override {
    lock_guard<mutex> lck(mtx_update);
    if (state == State::idle) {
      go_to_helper(x, y, theta, heading_rear);
      return true;
    } else {
      yWarning() << "The controller is busy";
      return false;
    }
  }

  /****************************************************************/
  bool go_to_wait(const double x, const double y, const double theta,
                  const bool heading_rear, const int timeout)override {
    mtx_update.lock();
    if (state == State::idle) {
      go_to_helper(x, y, theta, heading_rear);
      mtx_update.unlock();
      unique_lock<mutex> lck(mtx_nav_done);
      if (timeout > 0) {
        cv_nav_done.wait_for(lck, chrono::seconds(timeout));
      } else {
        cv_nav_done.wait(lck);
      }
      return true;
    } else {
      yWarning() << "The controller is busy";
      mtx_update.unlock();
      return false;
    }
  }

  /****************************************************************/
  bool track_skeleton(const string& skeleton_tag)override {
    lock_guard<mutex> lck(mtx_update);
    if (state == State::idle) {
      this->skeleton_tag = skeleton_tag;
      state = State::track;
      yInfo() << "Tracking skeleton" << this->skeleton_tag;
      return true;
    } else {
      yWarning() << "The controller is busy";
      return false;
    }
  }

  /****************************************************************/
  bool is_navigating()override {
    lock_guard<mutex> lck(mtx_update);
    return (state != State::idle);
  }

  /****************************************************************/
  bool stop()override {
    lock_guard<mutex> lck(mtx_update);
    robot_velocity.zero();
    robot_velocity.sendTo(navCtrlPort);
    skeleton_tag.clear();
    skeleton_location = numeric_limits<double>::quiet_NaN();
    skeleton.reset();
    state = State::idle;
    cv_nav_done.notify_all();
    yInfo() << "Navigation stopped";
    return true;
  }

  /****************************************************************/
  bool reset_odometry()override {
    lock_guard<mutex> lck(mtx_update);
    bool ret = false;
    if (state == State::idle) {
      Bottle cmd, rep;
      cmd.addString("reset_odometry");
      yInfo() << "Odometry reset";
      if (navCmdPort.write(cmd, rep)) {
        ret = (rep.size() > 0);
      }
    } else {
      yWarning() << "The controller is busy";
    }
    return ret;
  }

  /****************************************************************/
  string which_skeleton()override {
    lock_guard<mutex> lck(mtx_update);
    return skeleton_tag;
  }

  /****************************************************************/
  Property get_state()override {
    return publishState();
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

