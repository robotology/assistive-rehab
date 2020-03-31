// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility 
 * Authors: Valentina Vasco
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */
 
#ifndef GAZEBO_ASSISTIVEREHAB_TUGINTERFACE_HH
#define GAZEBO_ASSISTIVEREHAB_TUGINTERFACE_HH

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Actor.hh>

#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Matrix.h>

#include <include/tugserver.h>

namespace gazebo
{

class TugInterface : public ModelPlugin
{
private:
    TugServer server;
    event::ConnectionPtr updateConnection;

    yarp::os::RpcServer m_rpcport;
    yarp::os::Network   m_network;
    yarp::os::Property m_parameters;

    double m_lastUpdateTime;

    physics::WorldPtr world;
    physics::ActorPtr actor;
    physics::Actor::SkeletonAnimation_M skel_animations;
    double velocity;
    double tolerance;
    double t0;
    physics::TrajectoryInfoPtr trajectoryInfo;
    yarp::sig::Matrix waypoints;
    int waypoint_id;

    enum class State { started, finished, stopped, idle } state;

public:
    TugInterface();
    ~TugInterface();
    
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
    void OnUpdate(const gazebo::common::UpdateInfo & /*_info*/);
    double updateCurrPose(ignition::math::Pose3d &pose, const ignition::math::Vector3d &targ,
                          const double &dt, const double &t, const ignition::math::Vector3d &o,
                          const double &angle);
    
};

GZ_REGISTER_MODEL_PLUGIN(TugInterface)


}


#endif
