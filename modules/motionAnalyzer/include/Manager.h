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
    yarp::os::RpcClient actionPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopePort;

    yarp::os::ResourceFinder *rf;

    std::map<std::string,Metric*> motion_repertoire;

    int numKeypoints;
    std::vector< std::vector <std::pair<std::string,yarp::sig::Vector>> > all_keypoints;
    std::vector<double> time_samples;
    std::vector<double> ideal_samples;
    std::vector<yarp::sig::Vector > all_planes;
    assistive_rehab::SkeletonWaist skeletonIn;

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

    yarp::os::Mutex mutex;

    //thresholds for feedback for rom
    std::vector<std::string> joint_list;
    yarp::sig::Vector sx_thresh;
    yarp::sig::Vector sy_thresh;
    yarp::sig::Vector sz_thresh;
    yarp::sig::Vector range_freq;
    yarp::sig::Vector psd_thresh;

    //thresholds for feedback for ep
    double radius,inliers_thresh;
    int zscore_thresh;

    void init();
    bool loadInitialConf();
    bool loadMotionList();
    bool loadMetric(const std::string &metric_tag);
    std::vector<std::string> listMetrics();
    bool selectSkel(const std::string &skel_tag);
    std::string getMotionType();
    bool start();
    bool stop();

    bool writeStructToMat(const std::string& name, const std::vector< std::vector< std::pair<std::string,yarp::sig::Vector> > >& keypoints_skel, mat_t *matfp);
    bool writeStructToMat(const std::string& name, const Metric& metric, mat_t *matfp);
    bool writeKeypointsToFile(mat_t *matfp);
    void print(const std::vector< std::vector< std::pair<std::string,yarp::sig::Vector> > >& keypoints_skel);

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
