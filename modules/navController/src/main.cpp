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
#include <yarp/dev/OdometryData.h>
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
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace assistive_rehab;

/****************************************************************/
class Navigator : public RFModule, public navController_IDL {
  double period{0.0};
  double velocity_linear_magnitude{0.0};
  double velocity_angular_saturation{0.0};
  double angular_tolerance{0.0};
  double distance_target{0.0};
  double distance_hysteresis_low{0.0};
  double distance_hysteresis_high{0.0};
  bool distance_hysteresis_active{false};
  bool offline_mode{false}; // If true, baseControl is not supposed to run and navController
                            // will not check baseControl rpc port, but will just expect
                            // a port for odometry data streaming
  bool force_odometry_reset{false}; // If true, the first odometry value acquired will be used to reset
                                    // the odometry
  bool first_odom_data_arrived{false}; // once the first odometry value has been acquired, this value
                                      // will become false
  bool no_odometry_data{false}; // If true, the navController will not send the odometry data obtained
                                // from the navLocPort but, when asked, but will just return x=0
                                // y = 0 and theta = 0
  double base_stop_threshold{0.1};
  double target_theta{0.0};
  int heading{1};
  double time_to_stop{0.0};
  shared_ptr<parallelPID> pid_controller;
  shared_ptr<AWLinEstimator> velocity_estimator;

  shared_ptr<Skeleton> skeleton;
  string skeleton_tag{""};
  Vector skeleton_location{zeros(3)};

  RpcClient navCmdPort;
  RpcClient odomCmdPort; // To send the reset odometry command
                         // since now it has to be sent to odometry_nws_yarp
  BufferedPort<OdometryData> navLocPort;
  BufferedPort<Bottle> navCtrlPort;
  BufferedPort<Bottle> opcPort;
  BufferedPort<Property> statePort;
  RpcServer cmdPort;
  BufferedPort<Bottle> stateBasePort;

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
    Matrix H0{eye(4, 4)};
    Location() = default;
    Location(const double x_, const double y_, const double theta_) : x(x_), y(y_), theta(theta_) { }
    bool getFrom(BufferedPort<OdometryData>& navLocPort) {
      if (OdometryData* odom = navLocPort.read(false)) {
        x = odom->odom_x;
        y = odom->odom_y;
        theta = odom->odom_theta;
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
      b.addFloat64(x);
      b.addFloat64(0.0);
      b.addFloat64(theta);
      navCtrlPort.writeStrict();
    }
  } robot_velocity;

  /****************************************************************/
  bool attach(RpcServer& source)override {
    return yarp().attachAsServer(source);
  }

  /****************************************************************/
  bool configure(ResourceFinder& rf)override {
    period = rf.check("period", Value(0.05)).asFloat64();
    velocity_linear_magnitude = abs(rf.check("velocity-linear-magnitude", Value(0.3)).asFloat64());
    velocity_angular_saturation = abs(rf.check("velocity-angular-saturation", Value(20.0)).asFloat64());
    angular_tolerance = abs(rf.check("angular-tolerance", Value(5.0)).asFloat64());
    distance_target = abs(rf.check("distance-target", Value(2.0)).asFloat64());
    distance_hysteresis_low = abs(rf.check("distance-hysteresis-low", Value(0.2)).asFloat64());
    distance_hysteresis_high = abs(rf.check("distance-hysteresis-high", Value(0.3)).asFloat64());
    base_stop_threshold = rf.check("base-stop-threshold", Value(0.1)).asFloat64();
    // Offline mode parameter. If 1, this->offline_mode will become true.
    if(rf.check("offline_mode"))
    {
      offline_mode = rf.find("offline_mode").asInt32() == 1;
      // Force odometry reset parameter. If 1 and if this->offline_mode,
      // this->force_odometry_reset will become true.
      if(rf.check("force_odom_reset"))
      {
        force_odometry_reset = (rf.find("force_odom_reset").asInt32() == 1 && offline_mode);
      }
      // No odom data parameter. If 1 and if this->offline_mode,
      // this->no_odometry_data will become true.
      if(rf.check("no_odom_data"))
      {
        no_odometry_data = (rf.find("no_odom_data").asInt32() == 1 && offline_mode);
      }
    }

    if(!no_odometry_data) {navLocPort.open("/navController/base/loc:i");}
    if(!offline_mode)
    {
      navCmdPort.open("/navController/base/cmd:rpc");
      navCtrlPort.open("/navController/base/ctrl:o");
      odomCmdPort.open("/navController/base/odomCmd:o");
    }
    opcPort.open("/navController/opc:i");
    statePort.open("/navController/state:o");
    stateBasePort.open("/navController/base_state:i");
    cmdPort.open("/navController/rpc");
    attach(cmdPort);

    double Kp = 1.0;
    double Ki = 1.0;
    double Kd = 0.5;
    double w = 1.0;
    double N = 1.0;
    double Tt = 1.0;
    Matrix sat(1, 2);
    sat(0, 0) = -velocity_angular_saturation;
    sat(0, 1) = -sat(0, 0);
    pid_controller = shared_ptr<parallelPID>(new parallelPID(period, Kp * ones(1), Ki * ones(1), Kd * ones(1),
                                                             w * ones(1), w * ones(1), w * ones(1),
                                                             N * ones(1), Tt * ones(1), sat));
    velocity_estimator = shared_ptr<AWLinEstimator>(new AWLinEstimator(16, 0.05));

    if(no_odometry_data) { return true; }
    if (Network::connect("/odometry2D_nws_yarp/odometry:o", navLocPort.getName()))
    {
      if(navCmdPort.asPort().isOpen() && !navCtrlPort.isClosed()){
        if (Network::connect(navCmdPort.getName(), "/baseControl/rpc") && Network::connect(odomCmdPort.getName(), "/odometry2D_nws_yarp/rpc") &&
        Network::connect(navCtrlPort.getName(), "/baseControl/input/command:i"))
        {
          Bottle cmd, rep;
          cmd.addString("run");
          if (navCmdPort.write(cmd, rep)) {
            if (rep.size() > 0) {
              if (reset_odometry(0.0, 0.0, 0.0)) {
                return true;
              }
            }
          }
        }
      }
      return true;
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
    if(navCmdPort.asPort().isOpen()) {navCmdPort.write(cmd, rep);}
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
    if (!statePort.isClosed())
      statePort.close();
    if (!stateBasePort.isClosed())
      stateBasePort.close();
    if (cmdPort.asPort().isOpen())
      cmdPort.close();
    return true;
  }

  /****************************************************************/
  double getPeriod()override {
    return period;
  }

  /****************************************************************/
  void get_skeleton() {
    if (opcPort.getInputCount() > 0) {
      if (Bottle* b = opcPort.read(false)) {
        if (!b->get(1).isString()) {
          skeleton.reset();
          for (int i = 1; i < b->size(); i++) {
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
  Property publish_state() {
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

    Location loc = get_location(robot_location.H0 * get_matrix(robot_location));
    lb_rloc.addFloat64(loc.x);
    lb_rloc.addFloat64(loc.y);
    lb_rloc.addFloat64(loc.theta);
    p.put("robot-location", b_rloc.get(0));

    lb_rvel.addFloat64(robot_velocity.x);
    lb_rvel.addFloat64(robot_velocity.theta);
    p.put("robot-velocity", b_rvel.get(0));

    if ((state == State::nav_angular) ||
        (state == State::nav_linear)) {
      lb_tloc.addFloat64(target_location->x);
      lb_tloc.addFloat64(target_location->y);
      lb_tloc.addFloat64(target_location->theta);
      lb_tloc.addInt32(heading);
      p.put("target-location", b_tloc.get(0));
    }

    if (state == State::track) {
      p.put("skeleton-tag", skeleton_tag);
      lb_sloc.addFloat64(skeleton_location[0]);
      lb_sloc.addFloat64(skeleton_location[1]);
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
  Matrix get_matrix(const Location& loc) {
    Vector rot(4, 0.0);
    rot[2] = 1.0;
    rot[3] = CTRL_DEG2RAD * loc.theta;
    Matrix H = axis2dcm(rot);
    H(0, 3) = loc.x;
    H(1, 3) = loc.y;
    return H;
  }

  /****************************************************************/
  Location get_location(const Matrix& H) {
    Location loc;
    Vector rot = dcm2axis(H);
    loc.x = H(0, 3);
    loc.y = H(1, 3);
    loc.theta = CTRL_RAD2DEG * rot[3] * sign(rot[2]);
    return loc;
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
  bool are_wheels_moving() {
    Bottle *base_state = stateBasePort.read();
    Bottle *base_speed = base_state->get(2).asList();
    double vleft = base_speed->get(0).asFloat64();
    double vright = base_speed->get(1).asFloat64();
    if (fabs(vleft) < base_stop_threshold && fabs(vright) < base_stop_threshold)
    {
      yInfo() << "The robot has stopped";
      return false;
    }
    else
    {
      return true;
    }
  }

  /****************************************************************/
  bool updateModule()override {
    lock_guard<mutex> lck(mtx_update);

    if(!no_odometry_data) { robot_location.getFrom(navLocPort); }
    if(force_odometry_reset && !first_odom_data_arrived)
    {
      reset_odometry(robot_location.x,robot_location.y,robot_location.theta);
      first_odom_data_arrived = true;
    }
    get_skeleton();

    robot_velocity.zero();
    skeleton_location = numeric_limits<double>::quiet_NaN();

    if (state == State::track) {
      if (skeleton) {
        if ((*skeleton)[KeyPointTag::hip_center]->isUpdated()) {
          skeleton_location = (*skeleton)[KeyPointTag::hip_center]->getPoint();
          Vector pos = (robot_location.H0 * get_matrix(robot_location)).getCol(3);
          double e = norm(skeleton_location.subVector(0, 1) - pos.subVector(0, 1)) - distance_target;
          double abs_e = abs(e);
          double command = velocity_linear_magnitude * sign(e);
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
        robot_velocity.x = sign(dot(dir, e)) * velocity_linear_magnitude;
      } else {
        if (target_location + 1 != target_locations.end()) {
          ++target_location;
          target_theta = compute_target_theta();
        } else {
          target_theta = target_location->theta;
        }
        state = State::nav_angular;
        pid_controller->reset(zeros(1));
        yInfo() << "Starting nav_angular: reaching" << target_theta << "[deg]";
      }
    }

    if(!offline_mode) {robot_velocity.sendTo(navCtrlPort);}

    if (statePort.getOutputCount() > 0) {
      statePort.prepare() = publish_state();
      statePort.writeStrict();
    }

    return true;
  }

  /****************************************************************/
  void go_to_helper(const double x, const double y, const double theta,
                    const bool heading_rear) {
    heading = (heading_rear ? -1 : 1);
    yInfo() << "Navigating to (" << x << y << theta << ") heading =" << heading;
    Matrix H = get_matrix(Location(x, y, theta));
    generate_waypoints(get_location(SE3inv(robot_location.H0) * H));
    target_location = target_locations.begin();
    Vector e = compute_error(*target_location);
    target_theta = ((target_location + 1 == target_locations.end()) && (norm(e) < distance_hysteresis_low) ?
                    target_location->theta : compute_target_theta());
    pid_controller->reset(zeros(1));
    state = State::nav_angular;
    yInfo() << "Starting nav_angular: reaching" << target_theta << "[deg]";
  }

  /****************************************************************/
  bool go_to_dontwait(const double x, const double y, const double theta,
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
      bool ret = true;
      if (timeout > 0) {
        ret = (cv_nav_done.wait_for(lck, chrono::seconds(timeout)) == cv_status::no_timeout);
      } else {
        cv_nav_done.wait(lck);
      }
      return ret;
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
    if(!offline_mode) {robot_velocity.sendTo(navCtrlPort);}
    skeleton_tag.clear();
    skeleton_location = numeric_limits<double>::quiet_NaN();
    skeleton.reset();
    state = State::idle;
    cv_nav_done.notify_all();
    double tstop = Time::now();
    yInfo() << "Navigation stopped";
    if (stateBasePort.getInputCount() > 0) {
      while (are_wheels_moving()) {
        // yInfo() << "The robot has not stopped yet";
      }
      double tbase_stop = Time::now();
      time_to_stop = tbase_stop - tstop;
    }
    return true;
  }

  /****************************************************************/
  double get_time_to_stop()override {
    lock_guard<mutex> lck(mtx_update);
    return time_to_stop;
  }

  /****************************************************************/
  bool reset_odometry(const double x_0, const double y_0,
                      const double theta_0)override {
    lock_guard<mutex> lck(mtx_update);
    bool ret = false;
    if (state == State::idle) {
      Bottle cmd, rep;
      cmd.addString("reset_odometry_RPC");
      yInfo() << "Odometry reset";
      if(!offline_mode)
      {
        if (odomCmdPort.write(cmd, rep)) {
          ret = (rep.size() > 0);
          if (ret) {
            robot_location.H0 = get_matrix(Location(x_0, y_0, theta_0));
          }
        }
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
    return publish_state();
  }

  /****************************************************************/
  bool set_distance_target(const double dist)override {
    lock_guard<mutex> lck(mtx_update);
    distance_target=dist;
    return true;
  }

  /****************************************************************/
  bool set_linear_velocity(const double lin_vel)override {
    lock_guard<mutex> lck(mtx_update);
    velocity_linear_magnitude = lin_vel;
    return true;
  }

  /****************************************************************/
  double get_linear_velocity()override {
    return velocity_linear_magnitude;
  }

  /****************************************************************/
  bool set_angular_velocity(const double ang_vel)override {
    lock_guard<mutex> lck(mtx_update);
    velocity_angular_saturation = ang_vel;
    return true;
  }

  /****************************************************************/
  double get_angular_velocity()override {
    return velocity_angular_saturation;
  }

/****************************************************************/
public:

  /****************************************************************/
  Navigator() : state(State::idle) { }
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
