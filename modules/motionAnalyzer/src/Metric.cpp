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

Rom::Rom(const string &tag_joint_, const string &motion_type_,
         const unsigned int n_movements_, const double &min_, const double &max_)
{
    tag_joint = tag_joint_;
    motion_type = motion_type_;
    n_movements = n_movements_;
    min = min_;
    max = max_;
}

void Rom::print()
{
    yInfo() << "Tag joint = " << tag_joint;
    yInfo() << "Number of movements = " << n_movements;
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
