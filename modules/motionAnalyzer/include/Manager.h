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

#include <fstream>
#include <list>
#include <deque>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <AssistiveRehab/skeleton.h>

#include "Manager.h"
#include "Processor.h"
#include "Metric.h"
#include "src/motionAnalyzer_IDL.h"

#include "matio.h"

class Manager : public yarp::os::RFModule,
                public motionAnalyzer_IDL
{

    yarp::os::RpcClient opcPort;
    yarp::os::RpcServer rpcPort;
    yarp::os::RpcClient scalerPort;
    yarp::os::RpcClient dtwPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopePort;

    yarp::os::ResourceFinder *rf;

    std::map<std::string,Metric*> motion_repertoire;

    int numKeypoints;
    std::vector<std::pair<std::string,yarp::sig::Vector>> initial_keypoints;
    std::vector<std::pair<std::string,yarp::sig::Vector>> new_keypoints;
    std::vector< std::vector <std::pair<std::string,yarp::sig::Vector>> > all_keypoints;
    std::vector<double> time_samples;
    std::vector<double> ideal_samples;
    std::vector<yarp::sig::Vector > all_planes;
    assistive_rehab::SkeletonWaist* skeletonInit;
    assistive_rehab::SkeletonWaist* skel;
    std::vector<assistive_rehab::SkeletonWaist*> skeletonsInit;

    assistive_rehab::SkeletonWaist skeletonIn;

    std::map<std::string, std::pair<std::string,double>> keypoints2conf;

    yarp::sig::Vector elbowLeft_init;
    yarp::sig::Vector elbowRight_init;
    yarp::sig::Vector handLeft_init;
    yarp::sig::Vector handRight_init;
    yarp::sig::Vector head_init;
    yarp::sig::Vector shoulderCenter_init;
    yarp::sig::Vector shoulderLeft_init;
    yarp::sig::Vector shoulderRight_init;
    yarp::sig::Vector hipLeft_init;
    yarp::sig::Vector hipRight_init;
    yarp::sig::Vector kneeLeft_init;
    yarp::sig::Vector kneeRight_init;
    yarp::sig::Vector ankleLeft_init;
    yarp::sig::Vector ankleRight_init;
    yarp::sig::Vector hipCenter_init;

    yarp::sig::Vector elbowLeft;
    yarp::sig::Vector elbowRight;
    yarp::sig::Vector handLeft;
    yarp::sig::Vector handRight;
    yarp::sig::Vector head;
    yarp::sig::Vector shoulderCenter;
    yarp::sig::Vector shoulderLeft;
    yarp::sig::Vector shoulderRight;
    yarp::sig::Vector hipLeft;
    yarp::sig::Vector hipRight;
    yarp::sig::Vector kneeLeft;
    yarp::sig::Vector kneeRight;
    yarp::sig::Vector ankleLeft;
    yarp::sig::Vector ankleRight;

    yarp::sig::Vector cameraposinit;
    yarp::sig::Vector focalpointinit;

    Metric* metric_repertoire;
    Metric* metric;
    Processor* processor;

    double tstart;
    double tstart_session;
    double tend_session;
    bool finishedSession;

    bool starting;

    std::string skel_tag;

    int nsession;
    std::string out_folder;
    bool updated;
    double result;
    std::deque<double> result_time;

    double score_exercise;

    yarp::os::Mutex mutex;

    void init();
    bool loadInitialConf();
    bool loadInitialConf(const yarp::os::Bottle& b, assistive_rehab::SkeletonWaist *skeletonInit);
    bool loadMotionList();
    double loadMetric(const std::string &metric_tag);
    void getJointInitialConf(const yarp::os::Bottle &bMotion, const std::string &tag);
    double findMin();
    double findMax();
    std::vector<std::string> listMetrics();
    bool selectSkel(const std::string &skel_tag);
    double getQuality();
    bool start();
    bool startDebug();
    bool stop();

    bool writeStructToMat(const std::string& name, const std::vector< std::vector< std::pair<std::string,yarp::sig::Vector> > >& keypoints_skel, mat_t *matfp);
    bool writeStructToMat(const std::string& name, const Metric& metric, mat_t *matfp);
    bool writeKeypointsToFile(mat_t *matfp);
    void print(const std::vector< std::vector< std::pair<std::string,yarp::sig::Vector> > >& keypoints_skel);

    void getKeyframes();
    void getSkeleton();
    bool attach(yarp::os::RpcServer &source);

public:

    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    double getPeriod();
    bool updateModule();
    bool interruptModule();

};

#endif
