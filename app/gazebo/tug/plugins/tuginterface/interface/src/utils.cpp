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
std::map<double, ignition::math::Pose3d> createMap(const yarp::sig::Matrix &t,const double &vel)
{
    std::map<double, ignition::math::Pose3d> m;
    yarp::sig::Vector t0=t.getRow(0).subVector(0,2);
    double duration=0.0;
    for (int i=0; i<t.rows(); i++)
    {
        yarp::sig::Vector t1=t.getRow(i).subVector(0,2);
        yarp::sig::Vector o=t.getRow(i).subVector(3,5);
        double dist=yarp::math::norm(t1-t0);
        duration+=dist/vel;
        ignition::math::Pose3d p;
        p.Set(t1[0],t1[1],t1[2],o[0],o[1],o[2]);
        m[duration]=p;
        t0=t1;
    }
    return m;
}

/****************************************************************/
std::map<double, ignition::math::Pose3d> generateWaypoints(const int ntot, const double &vel,
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
        ignition::math::Pose3d target1=pIter->second;
        ignition::math::Vector3d e=target1.CoordPositionSub(target0);
        double norm_e=e.Length();
        ignition::math::Vector3d dir=e/norm_e;
        ignition::math::Pose3d v;
        double segment=norm_e/ntot;
        for (int step=0;step<=ntot-1;step++)
        {
            double seg=(step+1)*segment;
            ignition::math::Vector3d pos(target0.Pos().X()+seg*dir[0],
                    target0.Pos().Y()+seg*dir[1],
                    target0.Pos().Z()+seg*dir[2]);
            ignition::math::Vector3d ori=target1.Rot().Euler();
            v.Set(pos,ori);
            t+=segment/vel;
            mout[t]=v;
        }
        target0=target1;
    }
    return mout;
}


