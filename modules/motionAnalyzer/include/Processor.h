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
    yarp::sig::Matrix curr_frame;
    yarp::sig::Vector plane_normal;
    double t0;

public:
    Processor();
    virtual ~Processor() {;}
    void setInitialConf(assistive_rehab::SkeletonStd &skeleton_, yarp::sig::Matrix &T);
    void setStartingTime(const double &tnow);
    void update(assistive_rehab::SkeletonStd& curr_skeleton_);
    yarp::sig::Vector projectOnPlane(const yarp::sig::Vector &v,const yarp::sig::Vector &plane);
    void updateCurrentFrame(assistive_rehab::SkeletonStd &skeleton_);
    yarp::sig::Vector toCurrFrame(const std::string &tag);

    yarp::sig::Vector getPlaneNormal() const { return plane_normal; }
    virtual void estimate() = 0;
    virtual yarp::os::Property getResult() = 0;
    virtual std::string getProcessedMetric() const = 0;
    virtual void reset() = 0;
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
    void reset() override;
};

class Step_Processor : public Processor
{
    Step* step;
    yarp::sig::Vector feetdist;
    yarp::sig::Vector feetwidth;
    yarp::sig::Vector tdist;
    iCub::ctrl::MedianFilter* filter_dist;
    iCub::ctrl::MedianFilter* filter_width;
    double step_thresh,step_window,time_window;

    double steplen,prev_steplen;
    double steplen_raw, prev_steplen_raw;
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
    void reset() override;

    std::pair< std::deque<double>,std::deque<int> > findPeaks(const yarp::sig::Vector &d,
                                                              const double &minv);
    void estimateSpatialParams(const double &dist,const double &width);
    double estimateCadence();
    double estimateSpeed();

    double getStepLen() const { return steplen; }
    double getStepLenRaw() const { return steplen_raw; }
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
    void reset() override;

    double getVel() { return vel; }
    double getSmoothness() { return smoothness; }
    double getTrajectory(const yarp::sig::Vector &v);
};

#endif
