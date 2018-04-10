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
         const Vector &plane_normal_, const double &min_, const double &max_, const double &timeout_)
{
    name = name_; //"ROM";
    motion_type = motion_type_;
    tag_joint = tag_joint_;
    ref_dir = ref_dir_;
    plane_normal = plane_normal_;
    min = min_;
    max = max_;
    timeout = timeout_;
}

void Rom::print()
{
    yInfo() << "Metric = " << name;
    yInfo() << "Motion type = " << motion_type;
    yInfo() << "Tag joint = " << tag_joint;
    yInfo() << "Min = " << min;
    yInfo() << "Max = " << max;
    yInfo() << "Timeout = " << timeout;
}
