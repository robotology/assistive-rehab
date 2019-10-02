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
#include <fstream>

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>

#include <AssistiveRehab/skeleton.h>
#include "Metric.h"

class Processor;

Processor* createProcessor(const std::string& metric_type, const Metric *metric_);

class Processor
{
    yarp::sig::Matrix invT;

protected:
    assistive_rehab::SkeletonStd curr_skeleton,first_skeleton;
    yarp::sig::Matrix inv_reference_system,gaze_frame;
    yarp::sig::Vector plane_normal;
    double t0;

public:
    Processor();
    virtual ~Processor() {;}
    void setInitialConf(assistive_rehab::SkeletonStd &skeleton_, yarp::sig::Matrix &T);
    void update(assistive_rehab::SkeletonStd& curr_skeleton_, const yarp::sig::Matrix &gaze_frame);
    yarp::sig::Vector projectOnPlane(const yarp::sig::Vector &v,const yarp::sig::Vector &plane);
    yarp::sig::Vector getKeypointRoot(const std::string &tag);
    void stop();

    yarp::sig::Vector getPlaneNormal() const { return plane_normal; }
    virtual void estimate() = 0;
    virtual yarp::os::Property getResult() = 0;
    virtual std::string getProcessedMetric() const = 0;

};

class Rom_Processor : public Processor
{
    Rom* rom;
    double range,prev_range;

public:

    Rom_Processor();
    Rom_Processor(const Metric *rom_);
    void estimate() override;
    yarp::os::Property getResult() override;
    std::string getProcessedMetric() const { return rom->getParams().find("name").asString(); }

};

class Step_Processor : public Processor
{
    Step* step;
    yarp::sig::Vector feetdist;
    yarp::sig::Vector feetwidth;
    iCub::ctrl::Filter* filter_dist;
    iCub::ctrl::Filter* filter_width;

    double steplen,prev_steplen;
    double stepwidth,prev_stepwidth;
    int numsteps;
    double cadence,prev_cadence;
    double speed,prev_speed;

public:

    Step_Processor();
    Step_Processor(const Metric *step_);
    ~Step_Processor();

    void estimate() override;
    yarp::os::Property getResult() override;
    std::string getProcessedMetric() const { return step->getParams().find("name").asString(); }

    std::pair<std::vector<double>,std::vector<int> > findPeaks(const yarp::sig::Vector &d);
    void estimateSpatialParams(const yarp::sig::Vector &dist, const yarp::sig::Vector &width);
    double estimateCadence();
    double estimateSpeed();

    double getStepLen() const { return steplen; }
    double getStepWidth() const { return stepwidth; }
    double getNumSteps() const { return numsteps; }
    double getCadence() const { return cadence; }
    double getSpeed() const { return speed; }
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
    double vel,smoothness,est_traj;
    double prev_est_traj,prev_ideal_traj;
    double prev_vel,prev_smoothness;

public:

    EndPoint_Processor();
    EndPoint_Processor(const Metric *ep_);
    ~EndPoint_Processor();

    void estimate() override;
    yarp::os::Property getResult() override;
    std::string getProcessedMetric() const { return ep->getParams().find("name").asString(); }

    double getVel() { return vel; }
    double getSmoothness() { return smoothness; }
    double getTrajectory(const yarp::sig::Vector &v);

};

#endif
