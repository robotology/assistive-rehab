/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Metric.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include "Metric.h"

using namespace std;
using namespace yarp::sig;
using namespace assistive_rehab;

Metric::Metric()
{

}

Metric::~Metric()
{

}

void Metric::initialize(const string &name_, const string &motion_type_, const string &tag_joint_, const Vector &ref_dir_,
                        const string &tag_plane_, const double &min_, const double &max_, const double &duration_,
                        const Vector &camerapos_, const Vector &focalpoint_,
                        const vector<string> &relaxed_joints_, const Vector &dtw_thresh_, const Vector &mean_thresh_,
                        const Vector &sx_thresh_, const Vector &sy_thresh_, const Vector &sz_thresh_,
                        const Vector &f_static_, const Vector &range_freq_)
{
    name = name_;
    motion_type = motion_type_;
    tag_joint = tag_joint_;
    ref_dir = ref_dir_;
    tag_plane = tag_plane_;
    min = min_;
    max = max_;
    duration = duration_;
    camerapos = camerapos_;
    focalpoint = focalpoint_;
    relaxed_joints = relaxed_joints_;
    dtw_thresh = dtw_thresh_;
    mean_thresh = mean_thresh_;
    sx_thresh = sx_thresh_;
    sy_thresh = sy_thresh_;
    sz_thresh = sz_thresh_;
    f_static = f_static_;
    range_freq = range_freq_;
}

void Metric::print()
{   
    yInfo() << "Metric = " << name;
    yInfo() << "Motion type = " << motion_type;
    yInfo() << "Tag joint = " << tag_joint;
    yInfo() << "Relaxed joints = ";
    for(size_t i=0; i<relaxed_joints.size(); i++)
        cout << relaxed_joints[i] << " ";
    cout << endl;
}

/************************/
/*        ROM           */
/************************/
Rom::Rom()
{

}

/************************/
/*      END POINT       */
/************************/
EndPoint::EndPoint()
{

}

void EndPoint::setVel(const double &vel_)
{
    vel = vel_;
}

void EndPoint::setSmoothness(const double &smoothness_)
{
    smoothness = smoothness_;
}

void EndPoint::setTarget(const Vector &target_)
{
    target = target_;
}
