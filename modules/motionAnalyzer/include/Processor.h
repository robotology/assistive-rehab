/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Processor.h
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#ifndef __PROCESSOR_H__
#define __PROCESSOR_H__

#include <iostream>

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <math.h>

#include <AssistiveRehab/skeleton.h>
#include "Metric.h"

using namespace std;
using namespace yarp::sig;
using namespace assistive_rehab;

class Processor
{

    SkeletonStd skeleton_init;

public:
    Processor() {;}
    Processor(const SkeletonStd& skeleton_init_);
    virtual ~Processor() {;}
    virtual bool isDeviatingFromIntialPose(SkeletonStd& curr_skeleton);
    virtual bool isDeviatingFromIntialPose(const KeyPoint &keypoint, const KeyPoint &keypoint_init);
//    virtual string getMetricTag() {;}

};

class Rom_Processor : public Processor
{

    Rom *rom;

public:

    Rom_Processor(Rom *rom_);
    double computeRom();
//    virtual string getMetricTag();

};

#endif
