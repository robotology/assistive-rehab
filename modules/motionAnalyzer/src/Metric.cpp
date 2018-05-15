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

Metric::Metric()
{

}

Metric::~Metric()
{

}

void Metric::print()
{

}

Rom::Rom()
{

}

Rom::Rom(const string &name_, const string &motion_type_, const string &tag_joint_, const Vector &ref_dir_,
         const string &tag_plane_, const double &range_plane_, const double &min_, const double &max_,
         const double &duration_, const double &tempwin_, const double &threshold_, const Vector &camerapos_,
         const Vector &focalpoint_, const map<string, pair<string,double>> &keypoints2conf_)
{
    name = name_; //"ROM";
    motion_type = motion_type_;
    tag_joint = tag_joint_;
    ref_dir = ref_dir_;
    tag_plane = tag_plane_;
    range_plane = range_plane_;
    min = min_;
    max = max_;
    duration = duration_;
    tempwin = tempwin_;
    threshold = threshold_;
    camerapos = camerapos_;
    focalpoint = focalpoint_;
    keypoints2conf = keypoints2conf_;
}

void Rom::print()
{
    yInfo() << "Metric = " << name;
    yInfo() << "Motion type = " << motion_type;
    yInfo() << "Tag joint = " << tag_joint;
    yInfo() << "Min = " << min;
    yInfo() << "Max = " << max;
    yInfo() << "Duration = " << duration;
    yInfo() << "Temp win = " << tempwin;
    yInfo() << "Threshold = " << threshold;

    yInfo() << KeyPointTag::elbow_left << keypoints2conf[KeyPointTag::elbow_left].first;
    yInfo() << KeyPointTag::elbow_right << keypoints2conf[KeyPointTag::elbow_right].first;
    yInfo() << KeyPointTag::hand_left << keypoints2conf[KeyPointTag::hand_left].first;
    yInfo() << KeyPointTag::hand_right << keypoints2conf[KeyPointTag::hand_right].first;
    yInfo() << KeyPointTag::head << keypoints2conf[KeyPointTag::head].first;
    yInfo() << KeyPointTag::shoulder_center << keypoints2conf[KeyPointTag::shoulder_center].first;
    yInfo() << KeyPointTag::shoulder_left << keypoints2conf[KeyPointTag::shoulder_left].first;
    yInfo() << KeyPointTag::shoulder_right << keypoints2conf[KeyPointTag::shoulder_right].first;
    yInfo() << KeyPointTag::hip_left << keypoints2conf[KeyPointTag::hip_left].first;
    yInfo() << KeyPointTag::hip_right << keypoints2conf[KeyPointTag::hip_right].first;
    yInfo() << KeyPointTag::knee_left << keypoints2conf[KeyPointTag::knee_left].first;
    yInfo() << KeyPointTag::knee_right << keypoints2conf[KeyPointTag::knee_right].first;
    yInfo() << KeyPointTag::ankle_left << keypoints2conf[KeyPointTag::ankle_left].first;
    yInfo() << KeyPointTag::ankle_right << keypoints2conf[KeyPointTag::ankle_right].first;
}
