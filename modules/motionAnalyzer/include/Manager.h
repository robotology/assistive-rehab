/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Manager.h
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#ifndef __MANAGER_H__
#define __MANAGER_H__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <AssistiveRehab/skeleton.h>

#include "Manager.h"
#include "Processor.h"
#include "Metric.h"
#include "src/motionAnalyzer_IDL.h"

#include "matio.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class Manager : public RFModule,
                public motionAnalyzer_IDL
{

    RpcClient opcPort;
    RpcServer rpcPort;
    BufferedPort<Bottle> scopePort;
    BufferedPort<Bottle> testPort;

    ResourceFinder *rf;

    int nmovements;

    map<string,Metric*> motion_repertoire;

    int numKeypoints;
    vector<pair<string,Vector>> initial_keypoints;
    vector<pair<string,Vector>> curr_keypoints;
    vector< vector <pair<string,Vector>> > all_keypoints;
    vector<double> time_samples;
    SkeletonWaist initial_skeleton;

    map<string, pair<string,double>> keypoints2conf;

    Vector elbowLeft_init;
    Vector elbowRight_init;
    Vector handLeft_init;
    Vector handRight_init;
    Vector head_init;
    Vector shoulderCenter_init;
    Vector shoulderLeft_init;
    Vector shoulderRight_init;
    Vector hipLeft_init;
    Vector hipRight_init;
    Vector kneeLeft_init;
    Vector kneeRight_init;
    Vector ankleLeft_init;
    Vector ankleRight_init;

    Vector elbowLeft;
    Vector elbowRight;
    Vector handLeft;
    Vector handRight;
    Vector head;
    Vector shoulderCenter;
    Vector shoulderLeft;
    Vector shoulderRight;
    Vector hipLeft;
    Vector hipRight;
    Vector kneeLeft;
    Vector kneeRight;
    Vector ankleLeft;
    Vector ankleRight;

    vector<Metric*> metrics;
    vector<Processor*> processors;

    double tstart;
    double tstart_session;
    double tend_session;
    bool finishedSession;

    mat_t *matfp;
    string filename_report;

//    Metric* metric;
//    Processor* processor;

    void init();
    bool loadInitialConf(const string& motion_repertoire_file);
    bool loadInitialConf(const Bottle& b);
    bool loadMotionList(const string& motion_repertoire_file);
    bool loadSequence(const string& sequencer_file);
    bool exportData();

    bool writeStructToMat(const string& name, const vector< vector< pair<string,Vector> > >& keypoints_skel);
    bool writeStructToMat(const string& name, const Metric& metric);
    bool writeKeypointsToFile();
    void print(const vector< vector< pair<string,Vector> > >& keypoints_skel);

    void getKeyframes();
    bool attach(RpcServer &source);

public:

    bool configure(ResourceFinder &rf);
    bool close();
    double getPeriod();
    bool updateModule();
    bool interruptModule();

};

#endif
