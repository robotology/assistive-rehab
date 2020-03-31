// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Vasco
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <include/tugserver.h>

using namespace std;
using namespace gazebo;

/****************************************************************/
TugServer::TugServer() : starting(false), speed(0.0)
{

}

/****************************************************************/
TugServer::~TugServer()
{

}

/****************************************************************/
bool TugServer::start()
{
    actor->Play();
    starting=true;
    yInfo()<<"Playing script";
    return true;
}

/****************************************************************/
bool TugServer::stop()
{
    actor->Stop();
    starting=false;
    yInfo()<<"Stop script";
    return true;
}

/****************************************************************/
bool TugServer::setSpeed(const double speed)
{
    this->speed=speed;
    ignition::math::Vector3d sp=actor->WorldPose().Pos();
    yarp::sig::Vector t0(3,0.0);
    t0[0]=sp[0];
    t0[1]=sp[1];
    t0[2]=sp[2];
    double duration=0.0;
    for (int i=0; i<waypoints.rows(); i++)
    {
        yarp::sig::Vector t1=waypoints.getRow(i).subVector(0,2);
        duration+=yarp::math::norm(t1-t0)/this->speed;
        t0=t1;
    }
    trajectory->endTime=trajectory->startTime+duration;
    trajectory->duration=duration;
    actor->SetCustomTrajectory(trajectory);
    yInfo()<<"Setting speed to"<<speed;
    return true;
}

/****************************************************************/
double TugServer::getSpeed()
{
    return this->speed;
}

/****************************************************************/
void TugServer::update(bool &trigger, double &vel)
{
    trigger=starting;
    vel=speed;
}

/****************************************************************/
void TugServer::init(const double &speed, const physics::TrajectoryInfoPtr &trajectory,
                     const yarp::sig::Matrix &waypoints)
{
    this->trajectory=trajectory;
    this->speed=speed;
    this->waypoints=waypoints;
}

/****************************************************************/
void TugServer::setStarting(const bool &trigger)
{
    this->starting=trigger;
}





