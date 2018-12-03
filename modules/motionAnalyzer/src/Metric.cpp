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
                        const string &tag_plane_, const double &min_, const double &max_, const int &duration_,
                        const double &twarp_, const Vector &camerapos_, const Vector &focalpoint_,
                        const vector<string> &joint_list_)
{
    name = name_;
    motion_type = motion_type_;
    tag_joint = tag_joint_;
    ref_dir = ref_dir_;
    tag_plane = tag_plane_;
    min = min_;
    max = max_;
    duration = duration_;
    twarp = twarp_;
    camerapos = camerapos_;
    focalpoint = focalpoint_;
    joint_list = joint_list_;
}

void Metric::print()
{   
    yInfo() << "Metric = " << name;
    yInfo() << "Motion type = " << motion_type;
    yInfo() << "Tag joint = " << tag_joint;
    yInfo() << "Joint list for feedback = ";
    for(size_t i=0; i<joint_list.size(); i++)
        cout << joint_list[i] << " ";
    cout << endl;
}

/************************/
/*        ROM           */
/************************/
Rom::Rom()
{

}

void Rom::setFeedbackThresholds(const yarp::sig::Vector &sx_thresh_, const yarp::sig::Vector &sy_thresh_,
                                const yarp::sig::Vector &sz_thresh_, const yarp::sig::Vector &range_freq_,
                                const yarp::sig::Vector &psd_thresh_)
{
    sx_thresh = sx_thresh_;
    sy_thresh = sy_thresh_;
    sz_thresh = sz_thresh_;
    range_freq = range_freq_;
    psd_thresh = psd_thresh_;
}

Matrix Rom::getFeedbackThresholds()
{
    thresholds.resize(5,joint_list.size());
    for(size_t i=0; i<joint_list.size(); i++)
    {
        thresholds[0][i]=sx_thresh[i];
        thresholds[1][i]=sy_thresh[i];
        thresholds[2][i]=sz_thresh[i];
        thresholds[3][i]=range_freq[i];
        thresholds[4][i]=psd_thresh[i];
    }

    return thresholds;
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

void EndPoint::setFeedbackThresholds(const double &radius_,const int zscore_thresh_,const double &inliers_thresh_)
{
    radius = radius_;
    zscore_thresh = zscore_thresh_;
    inliers_thresh = inliers_thresh_;
}

void EndPoint::setTarget(const Vector &target_)
{
    target = target_;
}

yarp::sig::Matrix EndPoint::getFeedbackThresholds()
{
    thresholds.resize(3,joint_list.size());
    for(size_t i=0; i<joint_list.size(); i++)
    {
        thresholds[0][i]=radius;
        thresholds[1][i]=zscore_thresh;
        thresholds[2][i]=inliers_thresh;
    }

    return thresholds;
}
