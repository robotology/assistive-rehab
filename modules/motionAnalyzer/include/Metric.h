/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Metric.h
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#ifndef __METRIC_H__
#define __METRIC_H__

#include <iostream>
#include <map>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

#include <AssistiveRehab/skeleton.h>

class Metric
{
protected:
    std::string name;
    std::string motion_type;
    std::string tag_joint;
    std::string tag_plane;
    int duration;
    yarp::sig::Vector ref_dir;
    double min;
    double max;
    double twarp;
    yarp::sig::Vector camerapos;
    yarp::sig::Vector focalpoint;
    std::vector<std::string> joint_list;
    yarp::sig::Matrix thresholds;

public:
    Metric();
    virtual ~Metric();

    void print();
    void initialize(const std::string &name_, const std::string &motion_type_, const std::string &tag_joint_,
                    const yarp::sig::Vector &ref_dir_, const std::string &tag_plane_, const double &min_,
                    const double &max_, const int &duration_, const double &twarp_, const yarp::sig::Vector &camerapos_,
                    const yarp::sig::Vector &focalpoint_, const std::vector<std::string> &joint_list_);

    yarp::sig::Vector getRefDir() const { return ref_dir; }
    double getMax() const { return max; }
    double getMin() const { return min; }
    int getDuration() const { return duration; }
    double getTwarp() const { return twarp; }
    std::string getName() const { return name; }
    std::string getTagPlane() const { return tag_plane; }
    std::string getTagJoint() const { return tag_joint; }
    std::string getMotionType() const { return motion_type; }
    yarp::sig::Vector getCameraPos() const { return camerapos; }
    yarp::sig::Vector getFocalPoint() const { return focalpoint; }
    std::vector<std::string> getJoints() const { return joint_list; }

    virtual void setFeedbackThresholds(const yarp::sig::Vector &sx_thresh_, const yarp::sig::Vector &sy_thresh_,
                                       const yarp::sig::Vector &sz_thresh_, const yarp::sig::Vector &range_freq_,
                                       const yarp::sig::Vector &psd_thresh_) = 0;
    virtual void setFeedbackThresholds(const double &radius_, const int zscore_thresh_, const double &inliers_thresh_) = 0;
    virtual void setTarget(const yarp::sig::Vector &target_) = 0;
    virtual yarp::sig::Vector getTarget() = 0;
    virtual yarp::sig::Matrix getFeedbackThresholds() = 0;

};

class Rom : public Metric
{
    yarp::sig::Vector sx_thresh;
    yarp::sig::Vector sy_thresh;
    yarp::sig::Vector sz_thresh;
    yarp::sig::Vector range_freq;
    yarp::sig::Vector psd_thresh;

public:
    Rom();

    void setFeedbackThresholds(const yarp::sig::Vector &sx_thresh_, const yarp::sig::Vector &sy_thresh_,
                               const yarp::sig::Vector &sz_thresh_, const yarp::sig::Vector &range_freq_,
                               const yarp::sig::Vector &psd_thresh_);
    void setFeedbackThresholds(const double &radius_, const int zscore_thresh_, const double &inliers_thresh_) {;}
    void setTarget(const yarp::sig::Vector &target_) {;}

    yarp::sig::Vector getSxThresh() const { return sx_thresh; }
    yarp::sig::Vector getSyThresh() const { return sy_thresh; }
    yarp::sig::Vector getSzThresh() const { return sz_thresh; }
    yarp::sig::Vector getRangeFreq() const { return range_freq; }
    yarp::sig::Vector getPsdThresh() const { return psd_thresh; }

    yarp::sig::Matrix getFeedbackThresholds();
    yarp::sig::Vector getTarget() { return yarp::sig::Vector(3,0.0);}

};

class EndPoint : public Metric
{
    yarp::sig::Vector target;
    double radius,inliers_thresh;
    int zscore_thresh;
    double vel;
    double smoothness;

public:
    EndPoint();

    void setFeedbackThresholds(const yarp::sig::Vector &sx_thresh_, const yarp::sig::Vector &sy_thresh_,
                               const yarp::sig::Vector &sz_thresh_, const yarp::sig::Vector &range_freq_,
                               const yarp::sig::Vector &psd_thresh_) {;}
    void setFeedbackThresholds(const double &radius_, const int zscore_thresh_, const double &inliers_thresh_);
    void setTarget(const yarp::sig::Vector &target_);
    void setVel(const double &vel_);
    void setSmoothness(const double &smoothness_);
    double getVel() const { return vel; }
    double getSmoothness() const { return smoothness; }
    double getRadius() const { return radius; }
    int getZscoreThresh() const { return zscore_thresh; }
    double getInliersThresh() const { return inliers_thresh; }

    yarp::sig::Matrix getFeedbackThresholds();
    yarp::sig::Vector getTarget() { return target; }

};


#endif
