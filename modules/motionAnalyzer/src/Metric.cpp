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
                        const int &nrep_, const int &nenv_, const Vector &camerapos_, const Vector &focalpoint_)
{
    name = name_;
    motion_type = motion_type_;
    tag_joint = tag_joint_;
    ref_dir = ref_dir_;
    tag_plane = tag_plane_;
    min = min_;
    max = max_;
    duration = duration_;
    nrep = nrep_;
    nenv = nenv_;
    camerapos = camerapos_;
    focalpoint = focalpoint_;
}

void Metric::print()
{   
    yInfo() << "Metric = " << name;
    yInfo() << "Motion type = " << motion_type;
    yInfo() << "Tag joint = " << tag_joint;
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
