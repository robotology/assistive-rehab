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
    line_frame=yarp::math::zeros(4,4);
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
    actor->Stop();
    if (time > 0.0)
    {
        while(true)
        {
            if ((yarp::os::Time::now()-t0)>=time)
            {
                break;
            }
        }
        actor->PlayFromLastStop(true);
    }

    return true;
}

/****************************************************************/
bool TugServer::playFromLast(const bool complete)
{
    yInfo()<<"Playing from last";
    actor->PlayFromLastStop(complete);
    return true;
}

/****************************************************************/
string TugServer::getState()
{
    return actor->GetCurrentAnimation().first;
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
    actor->PlayWithAnimationName("walk");

    yInfo()<<"Going to"<<p.x<<p.y;
    return true;
}

/****************************************************************/
bool TugServer::goToWait(const Pose &p)
{
    goTo(p);
    while(true)
    {
        ignition::math::Pose3d cp=actor->WorldPose();
        double dist=sqrt((cp.Pos().X()-p.x)*(cp.Pos().X()-p.x)+
                (cp.Pos().Y()-p.y)*(cp.Pos().Y()-p.y));
        if (dist<=0.05)
        {
            break;
        }
    }
    return true;
}

/****************************************************************/
bool TugServer::goToSeq(const std::vector<double> &p)
{
    if (p.size()%3!=0)
    {
        yError()<<"Pose size must be multiple of 3";
        return false;
    }
    int i=0;
    while (i<p.size()-1)
    {
        Pose pi(p[i],p[i+1],p[i+2]);
        goToWait(pi);
        i=i+3;
    }
    return true;
}

/****************************************************************/
bool TugServer::setTarget(const Pose &p)
{
    ignition::math::Pose3d t(p.x,p.y,0.0,0.0,0.0,(M_PI/180.0)*p.theta);
    yarp::sig::Vector ty(3,0.0);
    ty[0]=t.Pos().X();
    ty[1]=t.Pos().Y();
    ty[2]=t.Rot().Yaw();
    targets.setRow(1,ty);
    updateMap(targets);
    yInfo()<<"Setting target to"<<ty.toString();
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
        actor->Play();
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
    if (animation.id>=0)
    {
        unsigned int id=animation.id;
        actor->PlayWithAnimationName(name,complete,id);
    }
    else
    {
        actor->PlayWithAnimationName(name,complete);
    }

    return true;
}

/****************************************************************/
yarp::os::Property TugServer::getModelPos(const string &model_name)
{
    yarp::os::Property prop;
    auto model=world->ModelByName(model_name);
    if (!model)
    {
        yError()<<model_name<<"does not exist";
        return prop;
    }

    ignition::math::Pose3d model_pose=model->WorldPose();
    ignition::math::Vector3d pos=model_pose.Pos();
    ignition::math::Vector3d angles=model_pose.Rot().Euler();

    yarp::sig::Vector model_pose_world({pos.X(),pos.Y(),pos.Z(),1.0,angles[0],angles[1],angles[2]});
    yarp::sig::Matrix model_rot=yarp::math::rpy2dcm(model_pose_world.subVector(4,6));
    yarp::sig::Vector tr=line_frame*model_pose_world.subVector(0,3);
    yarp::sig::Vector rot=yarp::math::dcm2axis(line_frame.submatrix(0,2,0,2)*model_rot);
    tr.pop_back();

    yarp::os::Property &subprop=prop.addGroup(model_name);
    yarp::os::Bottle b_pose;
    yarp::os::Bottle &tmp=b_pose.addList();
    tmp.addDouble(tr[0]);
    tmp.addDouble(tr[1]);
    tmp.addDouble(tr[2]);
    tmp.addDouble(rot[0]);
    tmp.addDouble(rot[1]);
    tmp.addDouble(rot[2]);
    tmp.addDouble(rot[3]);
    subprop.put("pose_world",b_pose.get(0));
    return prop;
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

/****************************************************************/
void TugServer::attachWorldPointer(gazebo::physics::WorldPtr p)
{
    world=p;
    auto model=world->ModelByName("start-line");
    ignition::math::Pose3d model_pose=model->WorldPose();
    ignition::math::Vector3d pos=model_pose.Pos();
    ignition::math::Vector3d angles=model_pose.Rot().Euler();
    yarp::sig::Vector model_pose_world({pos.X(),pos.Y(),pos.Z(),1.0,angles[0],angles[1],angles[2]});
    yarp::sig::Matrix model_rot=yarp::math::rpy2dcm(model_pose_world.subVector(4,6));
    yarp::sig::Matrix T1(4,4);
    yarp::sig::Vector v1=model_rot.subcol(0,0,4);
    if (yarp::math::norm(v1)>0.0)
    {
        v1/=yarp::math::norm(v1);
    }
    yarp::sig::Vector v2=model_rot.subcol(0,1,4);
    if (yarp::math::norm(v2)>0.0)
    {
        v2/=yarp::math::norm(v2);
    }
    yarp::sig::Vector v3=model_rot.subcol(0,2,4);
    if (yarp::math::norm(v3)>0.0)
    {
        v3/=yarp::math::norm(v3);
    }
    T1.setCol(0,v1);
    T1.setCol(1,v2);
    T1.setCol(2,v3);
    T1.setCol(3,model_pose_world.subVector(0,3));
    line_frame=yarp::math::SE3inv(T1);
}

/****************************************************************/
void TugServer::attachActorPointer(gazebo::physics::ActorPtr p)
{
    actor=p;
}





