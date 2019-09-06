/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Exercise.h
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#ifndef __EXERCISE_H__
#define __EXERCISE_H__

#include <Metric.h>

namespace ExerciseType
{
extern const std::string rehabilitation;
extern const std::string test;
}

namespace ExerciseTag
{
extern const std::string abduction_left;
extern const std::string internal_rotation_left;
extern const std::string external_rotation_left;
extern const std::string reaching_left;
extern const std::string tug;
}

class Exercise
{
protected:
    std::string name;
    std::string type;
    std::vector<const Metric*> metrics;
    yarp::sig::Matrix feedbackMat;
    yarp::os::Property feedparams;

public:
    Exercise();
    Exercise(const std::string &name_, const std::string &type_);

    void addMetric(const Metric *m);

    std::string getName() const { return name; }
    std::string getType() const { return type; }
    std::vector<const Metric*> getMetrics() const { return metrics; }
    const Metric* getCurrMetric(const std::string &metric_tag) const;
    std::vector<std::string> listMetrics() const;

    virtual void setFeedbackParams(const yarp::os::Property &p) = 0;
    yarp::os::Property getFeedbackParams() const { return feedparams; }
    virtual yarp::sig::Matrix getFeedbackThresholds() = 0;

    void print(std::ostream &os=std::cout);

    ~Exercise();

};

class RangeOfMotion : public Exercise
{
private:
    int duration;
    double twarp;
    std::vector<std::string> joint_list;
    yarp::sig::Vector sx_thresh;
    yarp::sig::Vector sy_thresh;
    yarp::sig::Vector sz_thresh;
    yarp::sig::Vector range_freq;
    yarp::sig::Vector psd_thresh;

public:
    RangeOfMotion(const std::string &name_);

    void setFeedbackParams(const yarp::os::Property &p) override;
    yarp::sig::Matrix getFeedbackThresholds() override;

};

//class AbductionLeft : public Exercise
//{
//private:
//    int duration;
//    double twarp;
//    std::vector<std::string> joint_list;
//    yarp::sig::Vector sx_thresh;
//    yarp::sig::Vector sy_thresh;
//    yarp::sig::Vector sz_thresh;
//    yarp::sig::Vector range_freq;
//    yarp::sig::Vector psd_thresh;

//public:
//    AbductionLeft();

//    void setFeedbackParams(const yarp::os::Property &p) override;
//    yarp::sig::Matrix getFeedbackThresholds() override;

//};

//class InternalRotationLeft : public Exercise
//{
//private:
//    int duration;
//    double twarp;
//    std::vector<std::string> joint_list;
//    yarp::sig::Vector sx_thresh;
//    yarp::sig::Vector sy_thresh;
//    yarp::sig::Vector sz_thresh;
//    yarp::sig::Vector range_freq;
//    yarp::sig::Vector psd_thresh;

//public:
//    InternalRotationLeft();

//    void setFeedbackParams(const yarp::os::Property &p) override;
//    yarp::sig::Matrix getFeedbackThresholds() override;

//};

//class ExternalRotationLeft : public Exercise
//{
//private:
//    int duration;
//    double twarp;
//    std::vector<std::string> joint_list;
//    yarp::sig::Vector sx_thresh;
//    yarp::sig::Vector sy_thresh;
//    yarp::sig::Vector sz_thresh;
//    yarp::sig::Vector range_freq;
//    yarp::sig::Vector psd_thresh;

//public:
//    ExternalRotationLeft();

//    void setFeedbackParams(const yarp::os::Property &p) override;
//    yarp::sig::Matrix getFeedbackThresholds() override;

//};


class ReachingLeft : public Exercise
{
private:
    int duration;
    double twarp;
    std::vector<std::string> joint_list;
    yarp::sig::Vector radius,inliers_thresh,zscore_thresh;

public:
    ReachingLeft();

    void setFeedbackParams(const yarp::os::Property &p) override;
    yarp::sig::Matrix getFeedbackThresholds() override;

};

class Tug : public Exercise
{

public:
    Tug();

    void setFeedbackParams(const yarp::os::Property &p) override {;}
    yarp::sig::Matrix getFeedbackThresholds() override {;}

};


#endif
