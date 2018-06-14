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

#include <AssistiveRehab/skeleton.h>
#include "Metric.h"

using namespace std;
using namespace yarp::sig;
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
    SkeletonWaist curr_skeleton,template_skeleton;
    Matrix inv_reference_system;
    Vector plane_normal;
    Vector coronal,sagittal,transverse;
    vector< vector< pair <string,Vector> >> devvector;

public:
    Processor();
    virtual ~Processor() {;}
//    virtual string getMotionType();
    void setInitialConf(SkeletonWaist *skeleton_init_, const map<string, pair<string, double> > &keypoints2conf_,
                        SkeletonWaist &skeleton);
    bool isStatic(const KeyPoint& keypoint);
    void update(SkeletonWaist& curr_skeleton_,SkeletonWaist& template_skeleton_);
    bool isDeviatingFromIntialPose();
    double isDeviatingFromIntialPose(const KeyPoint &keypoint, const KeyPoint &keypoint_init);
    double getDeviation() { return deviation; }
    void checkDeviation();
    vector<pair<string, vector<string> > > getFeedback();
    virtual double computeMetric(Vector &v1, Vector &plane_normal, Vector &ref_dir, double&check) { return 0.0; }
    virtual string getProcessedMetric() = 0;

};

class Rom_Processor : public Processor
{
    const Rom* rom;
    double prev_result, prev_score;

public:

    Rom_Processor();
    Rom_Processor(const Metric *rom_);
//    void setInitialConf(const SkeletonWaist& skeleton_init_, const map<string, pair<string,double>>& keypoints2conf_) override;
    double computeMetric(Vector &v1, Vector &plane_normal_, Vector &ref_dir, double &score_exercise);
    string getProcessedMetric() { return rom->getName(); }

    static const string motion_type;


};

#endif
