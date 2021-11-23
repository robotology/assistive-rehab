// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Vasco
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

#include <iostream>
#include <cmath>

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Actor.hh>

// gazebo yarp plugins
#include <GazeboYarpPlugins/common.h>
#include <GazeboYarpPlugins/Handler.hh>
#include <GazeboYarpPlugins/ConfHelpers.hh>

// ignition
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/common/Console.hh>

#include <include/utils.h>
#include <include/tuginterface.h>

using namespace gazebo;
using namespace std;

/****************************************************************/
TugInterface::TugInterface() : ModelPlugin()
{

}

/****************************************************************/
TugInterface::~TugInterface()
{
    m_rpcport.close();
}

/****************************************************************/
bool TugInterface::configure(const sdf::ElementPtr &_sdf)
{
    //Getting .ini configuration file from sdf
    if (_sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_name=_sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path=gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);
        if (ini_file_path!="" && m_parameters.fromConfigFile(ini_file_path.c_str()))
        {
            yInfo()<<"Found yarpConfigurationFile: loading from "<<ini_file_path;
        }
        else
        {
            yError() << "TugInterface::configure error could not load configuration file";
            return false;
        }
    }
    portname=m_parameters.find("name").asString();
    yarp::os::Bottle *tBottle=m_parameters.find("targets").asList();
    if (!tBottle->isNull())
    {
        int ncols=3;
        if ((tBottle->size()%ncols)==0)
        {
            int nrows=tBottle->size()/ncols;
            targets.resize(nrows,ncols);
            for (int i=0; i<nrows; i++)
            {
                for (int j=0; j<ncols; j++)
                {
                    targets[i][j]=tBottle->get(j+i*ncols).asFloat64();
                }
            }
        }
        else
        {
            yError()<<"Targets provided has"<<tBottle->size()<<"elements, but it must have a number of elements multiple of 3";
            return false;
        }
    }
    vel.lin_vel=m_parameters.find("linear-velocity").asFloat64();
    vel.ang_vel=m_parameters.find("angular-velocity").asFloat64();
    starting_animation=m_parameters.find("starting_animation").asString();
    return true;
}

/****************************************************************/
void TugInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError()<<"TugInterface::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    world=_model->GetWorld();
    actor=boost::dynamic_pointer_cast<physics::Actor>(_model);

    if (!configure(_sdf))
    {
        yError()<<"Could not configure plugin";
        return;
    }

    waypoints_map=createMap(targets,vel,walktime,nsteps);
    waypoints_map=generateWaypoints(vel,waypoints_map);

    sdf::ElementPtr world_sdf=world->SDF();
    sdf::ElementPtr actor_sdf=world_sdf->GetElement("actor");
    updateScript(actor_sdf,waypoints_map);
    actor->UpdateParameters(actor_sdf);

    auto animations=actor->SkeletonAnimations();
    if (!animations[starting_animation])
    {
        yWarning()<<starting_animation<<"not found";
    }
    else
    {
        actor->PlayWithAnimationName(starting_animation);
    }

    server.attachWorldPointer(world);
    server.attachActorPointer(actor);

    m_rpcport.open(portname);
    server.yarp().attachAsServer(m_rpcport);
    server.init(vel,targets,walktime,nsteps);

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection=event::Events::ConnectWorldUpdateBegin(
                boost::bind(&TugInterface::OnUpdate, this, _1));

}

/****************************************************************/
void TugInterface::OnUpdate(const common::UpdateInfo & _info)
{

}
