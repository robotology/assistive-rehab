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

Rom::Rom(const string &motion_type_, const string &tag_joint_, const Vector &ref_dir_,
         const Vector &plane_normal_, const double &min_, const double &max_)
{
    name = "ROM";
    motion_type = motion_type_;
    tag_joint = tag_joint_;
    ref_dir = ref_dir_;
    plane_normal = plane_normal_;
    min = min_;
    max = max_;
}

void Rom::print()
{
    yInfo() << "Metric = " << name;
    yInfo() << "Motion type = " << motion_type;
    yInfo() << "Tag joint = " << tag_joint;
    yInfo() << "Min = " << min;
    yInfo() << "Max = " << max;
//    yInfo() << "elbowLeft = (" << elbowLeft[0] << elbowLeft[1] << elbowLeft[2] << ")";
//    if(elbowLeft[3] == 0)
//        yInfo() << "stationary";
//    else
//        yInfo() << "mobile";
//    yInfo() << "elbowRight = (" << elbowRight[0] << elbowRight[1] << elbowRight[2] << ")";
//    if(elbowRight[3] == 0)
//        yInfo() << "stationary" << "\n";
//    else
//        yInfo() << "mobile" << "\n";
}
