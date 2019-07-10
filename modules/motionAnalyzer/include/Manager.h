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

#include <mutex>
#include <fstream>
#include <list>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <AssistiveRehab/skeleton.h>

#include "Manager.h"
#include "Processor.h"
#include "Metric.h"
#include "Exercise.h"
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

    std::map<std::string,Exercise*> motion_repertoire;

    std::vector< std::vector <std::pair<std::string,yarp::sig::Vector>> > all_keypoints;
    std::vector<double> time_samples;
    std::vector<yarp::sig::Vector > all_planes;
    assistive_rehab::SkeletonStd skeletonIn;

    yarp::sig::Vector cameraposinit;
    yarp::sig::Vector focalpointinit;

    std::vector<Exercise*> exercises;
    Exercise* curr_exercise;
    std::vector<Processor*> processors;
    const Metric* curr_metric;

    double tstart;
    double tstart_session;
    double tend_session;
    bool finishedSession;

    bool starting;

    std::string skel_tag,template_tag;
    bool use_robot_template,robot_skeleton_mirror;

    int nsession;
    std::string out_folder;
    bool updated;
    std::string prop_tag;
    std::mutex mtx;
    yarp::sig::Vector shoulder_height;
    iCub::ctrl::AWLinEstimator *lin_est_shoulder;

    bool loadMotionList(yarp::os::ResourceFinder &rf);
    bool loadExercise(const std::string &exercise_tag) override;
    std::vector<std::string> listExercises() override;
    std::vector<std::string> listMetricProps() override;
    std::vector<std::string> listJoints() override;
    bool selectSkel(const std::string &skel_tag) override;
    bool selectMetricProp(const std::string &prop_tag) override;
    bool selectMetric(const std::string &metric_tag) override;
    std::string getCurrMetricProp() override;
    std::vector<std::string> listMetrics() override;
    std::string getExercise() override;
    bool start(const bool use_robot_template) override;
    bool stop() override;
    bool setPart(const std::string &part) override;
    bool setTemplateTag(const std::string &template_tag) override;
    bool mirrorTemplate(const bool robot_skeleton_mirror) override;
    bool stopFeedback() override;
    bool isStanding(const double standing_thresh) override;
    bool isSitting(const double standing_thresh) override;
    bool hasCrossedFinishLine(const double finishline_thresh) override;

    bool writeStructToMat(const std::string& name, const std::vector< std::vector< std::pair<std::string,yarp::sig::Vector> > >& keypoints_skel, mat_t *matfp);
    bool writeStructToMat(const std::string& name, const Exercise *ex, mat_t *matfp);
    matvar_t * writeStructToMat(const Metric *m);
    bool writeKeypointsToFile(mat_t *matfp);
    void print(const std::vector< std::vector< std::pair<std::string,yarp::sig::Vector> > >& keypoints_skel);

    bool getLinePose(yarp::sig::Vector &line_pose);
    void getSkeleton();
    bool attach(yarp::os::RpcServer &source) override;

public:

    bool configure(yarp::os::ResourceFinder &rf) override;
    bool close() override;
    double getPeriod() override;
    bool updateModule() override;
    bool interruptModule() override;

};

#endif
