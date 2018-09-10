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
    double duration;
    int nrep;
    int nenv;
    yarp::sig::Vector ref_dir;
    double range_plane;
    double min;
    double max;
    double tempwin;
    double threshold;
    yarp::sig::Vector camerapos;
    yarp::sig::Vector focalpoint;
    std::map<std::string, std::pair<std::string,double>> keypoints2conf;

public:
    Metric();
    virtual ~Metric();

    void print();
    void initialize(const std::string &name_, const std::string &motion_type_, const std::string &tag_joint_, const yarp::sig::Vector &ref_dir_,
                    const std::string &tag_plane_, const double &range_plane_, const double &min_, const double &max_,
                    const double &duration_, const int &nrep_, const int & nenv_, const double &tempwin_,
                    const double &threshold_, const yarp::sig::Vector &camerapos_, const yarp::sig::Vector &focalpoint_,
                    const std::map<std::string, std::pair<std::string,double>> &keypoints2conf_);
    yarp::sig::Vector getRefDir() const { return ref_dir; }
    double getRangePlane() const { return range_plane; }
    double getMax() const { return max; }
    double getMin() const { return min; }
    double getDuration() const { return duration; }
    int getNrep() const { return nrep; }
    int getNenv() const { return nenv; }
    double getTempWin() const { return tempwin; }
    double getThresh() const { return threshold; }
    std::string getName() const { return name; }
    std::string getTagPlane() const { return tag_plane; }
    std::string getTagJoint() const { return tag_joint; }
    std::string getMotionType() const { return motion_type; }
    yarp::sig::Vector getCameraPos() const { return camerapos; }
    yarp::sig::Vector getFocalPoint() const { return focalpoint; }
    std::map<std::string, std::pair<std::string,double>> getInitialConf() const { return keypoints2conf; }

    virtual void setTarget(const yarp::sig::Vector &target_) = 0;
};

class Rom : public Metric
{

public:
    Rom();
    void setTarget(const yarp::sig::Vector &target_) {;}

};

class EndPoint : public Metric
{
    yarp::sig::Vector target;
    double vel;
    double smoothness;

public:
    EndPoint();

    void setTarget(const yarp::sig::Vector &target_);
    void setVel(const double &vel_);
    void setSmoothness(const double &smoothness_);
    double getVel() const { return vel; }
    double getSmoothness() const { return smoothness; }
    yarp::sig::Vector getTarget() const { return target; }

};


#endif
