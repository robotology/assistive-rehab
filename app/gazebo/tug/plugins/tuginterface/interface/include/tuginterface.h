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
    std::string portname;

    physics::WorldPtr world;
    physics::ActorPtr actor;
    double velocity;
    int numwaypoints;
    std::string starting_animation;
    yarp::sig::Matrix targets;
    std::map<double, ignition::math::Pose3d> waypoints_map;


public:
    TugInterface();
    ~TugInterface();
    
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/);
    void OnUpdate(const gazebo::common::UpdateInfo & /*_info*/);
    bool configure(const sdf::ElementPtr &/*_sdf*/);
    
};

GZ_REGISTER_MODEL_PLUGIN(TugInterface)


}


#endif
