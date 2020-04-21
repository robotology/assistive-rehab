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
TugServer::TugServer() : world(0), actor(0), vel(0.0,0.0)
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
bool TugServer::goTo(const Pose &p)
{
    ignition::math::Pose3d cp=actor->WorldPose();
    ignition::math::Pose3d t(p.x,p.y,0.0,0.0,0.0,(M_PI/180.0)*p.theta);

    yarp::sig::Vector cpy(3,0.0);
    cpy[0]=cp.Pos().X();
    cpy[1]=cp.Pos().Y();

    yarp::sig::Vector ty(3,0.0);
    ty[0]=t.Pos().X();
    ty[1]=t.Pos().Y();
    ty[2]=t.Rot().Yaw();

    yarp::sig::Matrix T(2,3);
    T.setRow(0,cpy);
    T.setRow(1,ty);

    updateMap(T);
    actor->Play("walk");

    yInfo()<<"Going to"<<p.x<<p.y;
    return true;
}

/****************************************************************/
bool TugServer::setSpeed(const double speed)
{
    this->vel.lin_vel=speed;
    updateMap(targets);
    yInfo()<<"Setting speed to"<<speed;
    return true;
}

/****************************************************************/
double TugServer::getSpeed()
{
    return this->vel.lin_vel;
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
bool TugServer::play(const Animation &animation, const bool complete)
{  
    updateMap(targets);
    physics::Actor::SkeletonAnimation_M skel_m=actor->SkeletonAnimations();
    string name=animation.name;
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
            yInfo()<<"Playing"<<name;
        }
    }
    int id=animation.id;
    actor->Play(name,complete,id);
    return true;
}

/****************************************************************/
void TugServer::updateMap(const yarp::sig::Matrix &t)
{
    std::map<double, ignition::math::Pose3d> wp_map=createMap(t,this->vel);
    wp_map=generateWaypoints(this->vel,wp_map);

    sdf::ElementPtr world_sdf=world->SDF();
    sdf::ElementPtr actor_sdf=world_sdf->GetElement("actor");
    updateScript(actor_sdf,wp_map);
    actor->UpdateParameters(actor_sdf);
}

/****************************************************************/
void TugServer::init(const Velocity &vel,
                     const yarp::sig::Matrix &targets)
{
    this->vel=vel;
    this->targets=targets;
}






