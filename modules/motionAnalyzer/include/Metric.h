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
    int n_movements;
    double min;
    double max;

    SkeletonStd skeleton;

public:
    Rom();
    Rom(const string &tag_joint_, const unsigned int id_joint_, const unsigned int n_movements_,
        const double &min_, const double &max_);
    bool update(const SkeletonStd& skeleton_);
    SkeletonStd getSkeleton() const { return skeleton; }

    int getIdJoint() const { return id_joint; }
    void print();

};

#endif
