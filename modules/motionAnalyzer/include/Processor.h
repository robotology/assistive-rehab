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
#include <map>
#include <cmath>

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include <AssistiveRehab/skeleton.h>
#include "Metric.h"

class Processor;

Processor* createProcessor(const std::string& motion_tag, const Metric* metric_);

class Processor
{
    yarp::sig::Matrix invT;

protected:
    assistive_rehab::SkeletonStd curr_skeleton,first_skeleton;
    yarp::sig::Matrix inv_reference_system;
    yarp::sig::Vector plane_normal,coronal,sagittal,transverse;

public:
    Processor();
    virtual ~Processor() {;}
    void setInitialConf(assistive_rehab::SkeletonStd &skeleton_, yarp::sig::Matrix &T);
    void update(assistive_rehab::SkeletonStd& curr_skeleton_);
    yarp::sig::Vector getPlaneNormal() const { return plane_normal; }
    virtual double computeMetric() = 0;
    virtual double getIdeal() = 0;
    virtual std::string getProcessedMetric() = 0;

};

class Rom_Processor : public Processor
{
    const Rom* rom;
    double prev_result;

public:

    Rom_Processor();
    Rom_Processor(const Metric *rom_);
    double computeMetric();
    double getIdeal() { return rom->getMax(); }
    std::string getProcessedMetric() { return rom->getName(); }

    static const std::string metric_tag;


};

class JerkEstimator : public iCub::ctrl::AWPolyEstimator
{
protected:
    virtual double getEsteeme() { return 6.0*coeff[3]; }

public:
    JerkEstimator(unsigned int _N, const double _D) : iCub::ctrl::AWPolyEstimator(3,_N,_D) { }
};

class EndPoint_Processor : public Processor
{
    EndPoint* ep;
    iCub::ctrl::AWLinEstimator *linEst;
    JerkEstimator *jerkEst;
    double ideal_traj;
    double prev_est_traj,prev_ideal_traj;
    double prev_vel,prev_smoothness;

public:

    EndPoint_Processor();
    EndPoint_Processor(const Metric *ep_);
    ~EndPoint_Processor();
    double computeMetric();
    void estimate();
    double getIdeal() { return 0.0; }
    std::string getProcessedMetric() { return ep->getName(); }
    double getVel() { return ep->getVel(); }
    double getSmoothness() { return ep->getSmoothness(); }
    double getTrajectory(const yarp::sig::Vector &v);

    static const std::string metric_tag;


};



#endif
