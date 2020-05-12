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

namespace MetricType
{
extern const std::string rom;
extern const std::string end_point;
extern const std::string step;
} 

class Metric
{
protected:
    std::string type;
    std::vector<std::string> properties;

public:
    Metric();
    virtual ~Metric();

    virtual void print(std::ostream &os=std::cout) const = 0;
    virtual yarp::os::Property getParams() const = 0;
    std::vector<std::string> getProperties() const { return properties; }

};

class Rom : public Metric
{
private:
    std::string name;
    std::string tag_joint;
    std::string tag_plane;
    yarp::sig::Vector ref_dir;
    std::string ref_joint;
    double minv,maxv;

public:
    Rom() {;}
    Rom(const std::string &type_,const std::string &name_, const std::string &tag_joint_,
        const std::string &tag_plane_, const yarp::sig::Vector &ref_dir_,
        const std::string &ref_joint_, const double &minv_, const double &maxv_);
    Rom(const Rom &r);
    Rom& operator = (const Rom &r);

    std::string getTagJoint() const { return tag_joint; }
    std::string getTagPlane() const { return tag_plane; }
    std::string getRefJoint() const { return ref_joint; }
    yarp::sig::Vector getRefDir() const { return ref_dir; }

    yarp::os::Property getParams() const override;
    void print(std::ostream &os=std::cout) const override;

};

class Step : public Metric
{
private:
    std::string name;
    yarp::sig::Vector num;
    yarp::sig::Vector den;
    double minv,maxv,thresh,step_window,time_window;

public:
    Step() {;}
    Step(const std::string &type_, const std::string &name_, const yarp::sig::Vector &num_,
         const yarp::sig::Vector &den_, const double &thresh_, const double &step_window_,
         const double &time_window_, const double &minv_, const double &maxv_);
    Step(const Step &r);
    Step& operator = (const Step &r);

    yarp::sig::Vector getNum() const { return num; }
    yarp::sig::Vector getDen() const { return den; }
    double getThresh() const { return thresh; }
    double getStepWindow() const { return step_window; }
    double getTimeWindow() const { return time_window; }

    yarp::os::Property getParams() const override;
    void print(std::ostream &os=std::cout) const override;

};

class EndPoint : public Metric
{

private:
    std::string name;
    std::string tag_joint;
    std::string tag_plane;
    yarp::sig::Vector ref_dir;
    double minv,maxv;
    yarp::sig::Vector target;

public:
    EndPoint() {;}
    EndPoint(const std::string &type_,const std::string &name_,const std::string &tag_joint_,
             const std::string &tag_plane_,const yarp::sig::Vector &ref_dir_,const double &minv_,const double &maxv_,
             const yarp::sig::Vector &target_);
    EndPoint(const EndPoint &ep);
    EndPoint& operator = (const EndPoint &ep);

    std::string getTagJoint() const { return tag_joint; }
    std::string getTagPlane() const { return tag_plane; }
    yarp::sig::Vector getRefDir() const { return ref_dir; }
    yarp::sig::Vector getTarget() const { return target; }

    yarp::os::Property getParams() const override;
    void print(std::ostream &os=std::cout) const override;

};

#endif
