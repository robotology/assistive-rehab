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

#include <map>

#include <include/utils.h>

using namespace std;
using namespace gazebo;

/****************************************************************/
void updateScript(sdf::ElementPtr &actor_sdf, const std::map<double, ignition::math::Pose3d> &m)
{
    sdf::ElementPtr script=actor_sdf->GetElement("script");
    sdf::ElementPtr traj=script->GetElement("trajectory");
    while (traj)
    {
        string anim_name=traj->Get<string>("type");
        if (anim_name=="walk")
        {
            traj->Clear();
            for (auto it=m.begin(); it!=m.end(); ++it)
            {
                sdf::ElementPtr wayp=traj->AddElement("waypoint");
                wayp->GetElement("time")->Set(it->first);
                wayp->GetElement("pose")->Set(it->second);
            }
        }
        traj=traj->GetNextElement("trajectory");
    }

    yInfo()<<"Adding to script:"<<script->ToString("");

}

/****************************************************************/
std::map<double, ignition::math::Pose3d> createMap(const yarp::sig::Matrix &t, const Velocity &vel, double &walktime, int &nsteps)
{
    std::map<double, ignition::math::Pose3d> m;
    yarp::sig::Vector t0=t.getRow(0);
    walktime=0.0;
    double duration=0.0;
    double step_length=0.643;
    nsteps=0;
    for (int i=0; i<t.rows(); i++)
    {
        yarp::sig::Vector t1=t.getRow(i);
        double dist=yarp::math::norm(t1.subVector(0,1)-t0.subVector(0,1));
        nsteps+=ceil(dist/step_length);
        if (dist>0.0)
        {
            duration+=dist/vel.lin_vel;
        }
        else
        {
            double angle=(180.0/M_PI)*fabs(t1[2]-t0[2]);
            duration+=angle/vel.ang_vel;
        }
        ignition::math::Pose3d p;
        p.Set(t1[0],t1[1],0.0,0.0,0.0,t1[2]);
        m.insert(std::pair<double, ignition::math::Pose3d>(duration,p));
        t0=t1;
    }
    yDebug()<<"updating map.. with duration"<<duration;
    yDebug()<<"number of steps"<<nsteps;
    walktime=duration;

    return m;
}

/****************************************************************/
std::map<double, ignition::math::Pose3d> generateWaypoints(const Velocity &vel,
                                                           const std::map<double, ignition::math::Pose3d> &m)
{
    std::map<double, ignition::math::Pose3d> mout;
    auto pFirst=m.begin();
    double t0=pFirst->first;
    ignition::math::Pose3d target0=pFirst->second;
    auto pIter=++m.begin();
    double t=t0;
    for (; pIter!=m.end(); pIter++)
    {
        auto it=m.find(t);
        if (it!=m.end())
        {
            mout[t]=it->second;
        }
        ignition::math::Pose3d target1=pIter->second;
        ignition::math::Vector3d e_pos=target1.Pos()-target0.Pos();
        double norm_epos=e_pos.Length();
        double prev_yaw;
        if (norm_epos>0.0) //target not reached
        {
            int ntot=ceil(norm_epos/0.5);
            ignition::math::Vector3d dir=e_pos/norm_epos;
            ignition::math::Pose3d v;
            double segment=norm_epos/ntot;
            prev_yaw=atan2(dir[1],dir[0]);
            for (int step=0;step<=ntot-1;step++)
            {
                double seg=(step+1)*segment;
                ignition::math::Vector3d pos(target0.Pos().X()+seg*dir[0],target0.Pos().Y()+seg*dir[1],0.0);
                ignition::math::Vector3d ori(0.0,0.0,prev_yaw);
                v.Set(pos,ori);
                t+=segment/vel.lin_vel;
                mout[t]=v;
            }
        }
        else
        {
            prev_yaw=target0.Rot().Yaw();
        }
        double delta=fabs(target1.Rot().Yaw()-prev_yaw);
        if (delta>0.0)
        {
            t+=(180.0/M_PI)*fabs(target1.Rot().Yaw())/vel.ang_vel;
            mout[t]=target1;
        }
        target0=target1;
    }
    return mout;
}


