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

#include <include/utils.h>
#include <include/tugserver.h>

using namespace std;
using namespace gazebo;

/****************************************************************/
TugServer::TugServer() : world(0), actor(0), numwaypoints(0), speed(0.0)
{

}

/****************************************************************/
TugServer::~TugServer()
{

}

/****************************************************************/
bool TugServer::stop()
{
    actor->Stop();
    yInfo()<<"Stop script";
    return true;
}

/****************************************************************/
bool TugServer::pause(const double time)
{
    yInfo()<<"Pausing for"<<time<<"seconds";
    double t0=yarp::os::Time::now();
    world->SetPaused(true);
    while(true)
    {
        if ((yarp::os::Time::now()-t0)>=time)
        {
            world->SetPaused(false);
            break;
        }
    }
    return true;
}

/****************************************************************/
bool TugServer::setSpeed(const double speed)
{
    this->speed=speed;
    wp_map.clear();
    wp_map=createMap(targets,speed);
    wp_map=generateWaypoints(numwaypoints,speed,wp_map);

    sdf::ElementPtr world_sdf=world->SDF();
    sdf::ElementPtr actor_sdf=world_sdf->GetElement("actor");
    updateScript(actor_sdf,wp_map);
    actor->UpdateParameters(actor_sdf);

    yInfo()<<"Setting speed to"<<speed;
    return true;
}

/****************************************************************/
double TugServer::getSpeed()
{
    return this->speed;
}

/****************************************************************/
std::vector<std::string> TugServer::getAnimationList()
{
    physics::Actor::SkeletonAnimation_M skel_m=actor->SkeletonAnimations();
    std::vector<std::string> animations;
    for (auto it=skel_m.begin(); it!=skel_m.end(); it++)
    {
        animations.push_back(it->first);
    }
    return animations;
}

/****************************************************************/
bool TugServer::play(const std::string &name, const bool complete, const int id)
{  
    physics::Actor::SkeletonAnimation_M skel_m=actor->SkeletonAnimations();
    if (!name.empty() && !skel_m[name])
    {
        yError() << "Animation not found";
        return false;
    }
    if (name.empty())
    {
        yInfo()<<"Playing whole script";
    }
    else
    {
        if (complete)
        {
            yInfo()<<"Playing script starting from"<<name;
        }
        else
        {
            yInfo()<<"Playing only"<<name;
        }
    }
    actor->Play(name,complete,id);
    return true;
}

/****************************************************************/
void TugServer::init(const double &speed, const int numwaypoints,
                     const yarp::sig::Matrix &targets)
{
    this->speed=speed;
    this->numwaypoints=numwaypoints;
    this->targets=targets;
}






