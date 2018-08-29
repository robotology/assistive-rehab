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

using namespace std;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace assistive_rehab;

class Processor;

Processor* createProcessor(const string& motion_tag, const Metric* metric_);

class Processor
{
    double deviation;
    Matrix invT;

protected:
    SkeletonWaist *skeleton_init;
    map<string, pair<string,double>> keypoints2conf;
    SkeletonWaist curr_skeleton,first_skeleton;
    Matrix inv_reference_system;
    Vector plane_normal,coronal,sagittal,transverse;
    double score_exercise;

public:
    Processor();
    virtual ~Processor() {;}
//    virtual string getMotionType();
    void setInitialConf(SkeletonWaist *skeleton_init_, const map<string, pair<string, double> > &keypoints2conf_,
                        SkeletonWaist &skeleton_);
    bool isStatic(const KeyPoint& keypoint);
    void update(SkeletonWaist& curr_skeleton_);
    bool isDeviatingFromIntialPose();
    double isDeviatingFromIntialPose(const KeyPoint &keypoint, const KeyPoint &keypoint_init);
    double getDeviation() { return deviation; }
    double getScoreExercise() const { return score_exercise; }
    Vector getPlaneNormal() const { return plane_normal; }
    virtual double computeMetric() = 0;
    virtual double getIdeal() = 0;
    virtual string getProcessedMetric() = 0;

};

class Rom_Processor : public Processor
{
    const Rom* rom;
    double prev_result, prev_score;

public:

    Rom_Processor();
    Rom_Processor(const Metric *rom_);
    double computeMetric();
    double getIdeal() { return rom->getMax(); }
    string getProcessedMetric() { return rom->getName(); }

    static const string metric_tag;


};

class AWThirdEstimator : public AWPolyEstimator
{
protected:
    virtual double getEsteeme() { return 3.0*coeff[3]; }

public:
    AWThirdEstimator(unsigned int _N, const double _D) : AWPolyEstimator(3,_N,_D) { }
};

class EndPoint_Processor : public Processor
{
    EndPoint* ep;
    AWLinEstimator *linEst;
    AWThirdEstimator *polyEst;
    double ideal_traj;
    double prev_est_traj,prev_ideal_traj;
    double prev_vel,prev_smoothness;

public:

    EndPoint_Processor();
    EndPoint_Processor(const Metric *ep_);
    ~EndPoint_Processor();
    double computeMetric();
    void estimate();
    double getIdeal() { return ideal_traj; }
    string getProcessedMetric() { return ep->getName(); }
    double getVel() { return ep->getVel(); }
    double getSmoothness() { return ep->getSmoothness(); }
    double getTrajectory(const Vector &k, const Vector &kref);

    static const string metric_tag;


};



#endif
