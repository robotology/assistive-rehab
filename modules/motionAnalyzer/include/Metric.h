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

using namespace std;
using namespace yarp::sig;
using namespace assistive_rehab;

class Metric
{
protected:
    string name;
    string motion_type;
    string tag_joint;
    string tag_plane;
    double duration;
    int nrep;
    int nenv;
    Vector camerapos;
    Vector focalpoint;
    map<string, pair<string,double>> keypoints2conf;

public:
    Metric();
    virtual ~Metric();
    void print();
    virtual Vector getRefDir() const = 0;
    virtual double getRangePlane() const = 0;
    virtual double getMax() const = 0;
    virtual double getMin() const = 0;
    virtual double getDuration() const = 0;
    virtual int getNrep() const = 0;
    virtual int getNenv() const = 0;
    virtual double getTempWin() const = 0;
    virtual double getThresh() const = 0;

    string getName() const { return name; }
    string getTagPlane() const { return tag_plane; }
    string getTagJoint() const { return tag_joint; }
    string getMotionType() const { return motion_type; }
    Vector getCameraPos() const { return camerapos; }
    Vector getFocalPoint() const { return focalpoint; }
    map<string, pair<string,double>> getInitialConf() const { return keypoints2conf; }
};

class Rom : public Metric
{
    Vector ref_dir;
    double range_plane;
    double min;
    double max;
    double tempwin;
    double threshold;

public:
    Rom();
    Rom(const string &name_, const string &motion_type_, const string &tag_joint_, const Vector &ref_dir_,
        const string &tag_plane_, const double &range_plane_, const double &min_, const double &max_,
        const double &duration_, const int &nrep_, const int & nenv_, const double &tempwin_,
        const double &threshold_, const Vector &camerapos_, const Vector &focalpoint_,
        const map<string, pair<string,double>> &keypoints2conf_);

    Vector getRefDir() const { return ref_dir; }
    double getRangePlane() const { return range_plane; }
    double getMax() const { return max; }
    double getMin() const { return min; }
    double getDuration() const { return duration; }
    int getNrep() const { return nrep; }
    int getNenv() const { return nenv; }
    double getTempWin() const { return tempwin; }
    double getThresh() const { return threshold; }

};

class EndPoint : public Metric
{
    Vector ref_dir;
    double range_plane;
    double min;
    double max;
    double tempwin;
    double threshold;
    Vector target;

    double vel;
    double smoothness;

public:
    EndPoint();
    EndPoint(const string &name_, const string &motion_type_, const string &tag_joint_, const Vector &ref_dir_,
             const string &tag_plane_, const double &range_plane_, const double &min_, const double &max_,
             const double &duration_, const int &nrep_, const int & nenv_, const double &tempwin_,
             const double &threshold_, const Vector &camerapos_, const Vector &focalpoint_,
             const map<string, pair<string,double>> &keypoints2conf_, const Vector &target_);

    Vector getRefDir() const { return ref_dir; }
    double getRangePlane() const { return range_plane; }
    double getMax() const { return max; }
    double getMin() const { return min; }
    double getDuration() const { return duration; }
    int getNrep() const { return nrep; }
    int getNenv() const { return nenv; }
    double getTempWin() const { return tempwin; }
    double getThresh() const { return threshold; }

    void setVel(const double &vel_);
    void setSmoothness(const double &smoothness_);
    double getVel() const { return vel; }
    double getSmoothness() const { return smoothness; }
    Vector getTarget() const { return target; }
};


#endif
