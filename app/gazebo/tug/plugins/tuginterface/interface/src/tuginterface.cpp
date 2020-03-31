// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Vasco
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

#include <iostream>

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
void TugInterface::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    if (!yarp::os::Network::checkNetwork(GazeboYarpPlugins::yarpNetworkInitializationTimeout))
    {
        yError() << "TugInterface::Load error: yarp network does not seem to be available, is the yarpserver running?";
        return;
    }

    //setting up proxy
    world=_model->GetWorld();
    actor=boost::dynamic_pointer_cast<physics::Actor>(_model);

    server.attachWorldPointer(world);
    server.attachActorPointer(actor);

    //Getting .ini configuration file from sdf
    bool configuration_loaded = false;
    if (_sdf->HasElement("yarpConfigurationFile"))
    {
        std::string ini_file_name=_sdf->Get<std::string>("yarpConfigurationFile");
        std::string ini_file_path=gazebo::common::SystemPaths::Instance()->FindFileURI(ini_file_name);

        if (ini_file_path!="" && m_parameters.fromConfigFile(ini_file_path.c_str()))
        {
            yInfo() << "Found yarpConfigurationFile: loading from " << ini_file_path ;
            configuration_loaded = true;
        }
    }

    if (!configuration_loaded)
    {
        yError() << "TugInterface::Load error could not load configuration file";
        return;
    }

    std::string portname=m_parameters.find("name").asString();
    yarp::os::Bottle *tBottle=m_parameters.find("waypoints").asList();
    if (!tBottle->isNull())
    {
        int ncols=7;
        int nrows=tBottle->size()/ncols;
        waypoints.resize(nrows,ncols);
        for (int i=0; i<nrows; i++)
        {
            for (int j=0; j<ncols; j++)
            {
                waypoints[i][j]=tBottle->get(j+i*ncols).asDouble();
            }
        }
    }

    velocity=m_parameters.find("velocity").asDouble();
    tolerance=m_parameters.find("tolerance").asDouble();

    m_lastUpdateTime=world->SimTime().Double();

    skel_animations=actor->SkeletonAnimations();
    yDebug()<<__LINE__<<"stand up"<<skel_animations["stand_up"]->GetLength();
    yDebug()<<__LINE__<<"walk"<<skel_animations["walk"]->GetLength();
    yDebug()<<__LINE__<<"sit down"<<skel_animations["sit_down"]->GetLength();

    ignition::math::Pose3d sp;
    sp.Pos().X()=0.0;
    sp.Pos().Y()=0.0;
    sp.Pos().Z()=1.0;
    actor->SetWorldPose(sp,false,false);
    yarp::sig::Vector t0(3,0.0);
    t0[0]=sp.Pos().X();
    t0[1]=sp.Pos().Y();
    t0[2]=sp.Pos().Z();
    double duration=0.0;
    for (int i=0; i<waypoints.rows(); i++)
    {
        yarp::sig::Vector t1=waypoints.getRow(i).subVector(0,2);
        duration+=yarp::math::norm(t1-t0)/velocity;
        t0=t1;
    }

    trajectoryInfo.reset(new physics::TrajectoryInfo());
    trajectoryInfo->type="walk";
    trajectoryInfo->id=1;
    trajectoryInfo->startTime=skel_animations["stand_up"]->GetLength();
    trajectoryInfo->endTime=trajectoryInfo->startTime+duration;
    trajectoryInfo->duration=duration;
    actor->SetCustomTrajectory(trajectoryInfo);

    m_rpcport.open(portname);
    server.yarp().attachAsServer(m_rpcport);
    server.init(velocity,trajectoryInfo,waypoints);

    state=State::idle;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection=event::Events::ConnectWorldUpdateBegin(
                boost::bind(&TugInterface::OnUpdate, this, _1));

}

/****************************************************************/
void TugInterface::OnUpdate(const common::UpdateInfo & _info)
{
    bool start;
    server.update(start,velocity);

    // Time delta
    double t=_info.simTime.Double();
    double dt=t-m_lastUpdateTime;

    if (!start)
    {
        state=State::idle;
    }
    else
    {
        if(state>=State::idle)
        {
            t0=t;
            waypoint_id=0;
            state=State::started;
            yDebug()<<"Started at"<<t0;
        }
    }

    if (state==State::started)
    {
        ignition::math::Pose3d currPose=actor->WorldPose();
        if ( (t-t0)>=trajectoryInfo->startTime &&
             (t-t0)<=trajectoryInfo->endTime )
        {
            yarp::sig::Vector currT=waypoints.getRow(waypoint_id);
            ignition::math::Vector3d target(currT[0],currT[1],currT[2]);
            ignition::math::Vector3d ori(currT[3],currT[4],currT[5]);
            double theta=currT[6];
            double dist=updateCurrPose(currPose,target,dt,t,ori,theta);
            if (dist<=tolerance)
            {
                yInfo()<<"Reached waypoint"<<waypoint_id<<"in"<<t-t0-trajectoryInfo->startTime;
                waypoint_id++;
                if (waypoint_id>=waypoints.rows())
                {
                    yInfo()<<"Reached all waypoints";
                    state=State::finished;
                }
            }
        }
    }

    if (state==State::finished)
    {
        if ( (t-t0)>=actor->ScriptLength() )
        {
            yInfo()<<"Done";
            state=State::idle;
            server.setStarting(false);
        }
    }

    m_lastUpdateTime=t;
}

/****************************************************************/
double TugInterface::updateCurrPose(ignition::math::Pose3d &pose,
                                    const ignition::math::Vector3d &targ,
                                    const double &dt, const double &t,
                                    const ignition::math::Vector3d &o,
                                    const double &angle)
{
    // Compute distance from target
    ignition::math::Vector3d distance=targ-pose.Pos();
    double dist=distance.Length();
    yarp::sig::Vector d(3,0.0);
    d[0]=distance[0];
    d[1]=distance[1];
    d[2]=distance[2];
    yarp::sig::Vector e=yarp::math::sign(d);
    ignition::math::Vector3d ei(e[0],e[1],e[2]);

    // Estimate current pose
    pose.Pos()+=ei*velocity*dt; //distance*velocity*(dt/3.7);
    pose.Pos().Z()=1.0; //offset (canonical link is in the hip)
    pose.Rot().Axis(o,angle);

    actor->SetWorldPose(pose,false,false);
    // When a custom trajectory is defined,
    // the script time must be set to play the animation.
    this->actor->SetScriptTime(t);

    return dist;
}



