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
    string motion_type;
    string tag_joint;
    Vector ref_dir;
    Vector plane_normal;
    double min;
    double max;

public:
    Rom();
    Rom(const string &motion_type_, const string &tag_joint_, const Vector &ref_dir_,
        const Vector &plane_normal_, const double &min_, const double &max_);

    string getTagJoint() const { return tag_joint; }
    string getMotionType() const { return motion_type; }
    Vector getRefDir() const { return ref_dir; }
    Vector getPlane() const { return plane_normal; }
    void print();

};

#endif
