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
    yarp::os::RpcClient navPort;
    yarp::os::BufferedPort<yarp::os::Bottle> scopePort;

    enum class State { idle, sitting, standing, crossed, out_of_bounds} state;
    bool standing;

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

    double period;

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
    double shoulder_center_height_vel,standing_thresh,finishline_thresh;
    double _max_finish_line_overrun, _max_reasonable_ankles_dist;
    std::vector<double> line_pose;
    yarp::sig::Matrix world_frame;

    yarp::os::Bottle bResult;

    bool loadMotionList(yarp::os::ResourceFinder &rf);
    yarp::os::Property loadFeedbackList(const yarp::os::Bottle &bExercise, const std::string &ex_tag);
    Metric *loadMetricsList(const yarp::os::Bottle &bMetricEx, const std::string &metric_tag,
                            const std::string &metric_type);
    Exercise* loadExerciseList(const yarp::os::Bottle &bGeneral, const std::string &ex_tag);
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
    bool runScaler();
    bool runActionRecognizer(const yarp::sig::Matrix &T);
    bool runDtw(const yarp::sig::Matrix &T);
    bool start(const bool use_robot_template) override;
    bool stop() override;
    bool setPart(const std::string &part) override;
    bool setTemplateTag(const std::string &template_tag) override;
    bool mirrorTemplate(const bool robot_skeleton_mirror) override;
    bool stopFeedback() override;
    bool setLinePose(const std::vector<double> &line_pose) override;
    yarp::os::Property getState() override;

    bool writeStructToMat(const std::string& name, const std::vector< std::vector< std::pair<std::string,yarp::sig::Vector> > >& keypoints_skel, mat_t *matfp);
    bool writeStructToMat(const std::string& name, const Exercise *ex, mat_t *matfp);
    void createSubfield(matvar_t *submatvar, double *val, size_t *dims, const char *name);
    void createSubfield(matvar_t *submatvar, const std::string &val_str, size_t *dims, const char *name);
    matvar_t *createRomField(const yarp::os::Property &params);
    matvar_t* createStepField(const yarp::os::Property &params);
    matvar_t* createEpField(const yarp::os::Property &params);
    matvar_t * writeStructToMat(const Metric *m);
    bool writeKeypointsToFile(mat_t *matfp);
    void print(const std::vector< std::vector< std::pair<std::string,yarp::sig::Vector> > >& keypoints_skel);

    void getSkeleton();
    bool isStanding();
    bool isSitting();
    bool hasCrossedLine(std::vector<double>& line);
    bool hasCrossedFinishLine();
    bool hasMovedAwayFromFinishLine();
    yarp::os::Property publishState();
    void updateState();
    void estimate();
    void writeMatio();
    void reset();
    bool attach(yarp::os::RpcServer &source) override;

public:

    bool configure(yarp::os::ResourceFinder &rf) override;
    bool close() override;
    double getPeriod() override;
    bool updateModule() override;
    bool interruptModule() override;

};

#endif
