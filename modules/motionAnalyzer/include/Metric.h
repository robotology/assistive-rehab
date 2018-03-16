/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Metric.h
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#ifndef __METRIC_H__
#define __METRIC_H__

#include <iostream>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

#include <AssistiveRehab/skeleton.h>

using namespace std;
using namespace yarp::sig;
using namespace assistive_rehab;

class Metric
{

public:
    Metric() {;}
    virtual ~Metric() {;}
    virtual void print() = 0;
};

class Rom : public Metric
{
    string tag_joint;
    int id_joint;
    string motion_type;
    int n_movements;
    double min;
    double max;

public:
    Rom();
    Rom(const string &tag_joint_, const unsigned int id_joint_, const string &motion_type_,
        const unsigned int n_movements_, const double &min_, const double &max_);

    int getIdJoint() const { return id_joint; }
    string getTagJoint() const { return tag_joint; }
    string getMotionType() const { return motion_type; }
    void print();

};

#endif
