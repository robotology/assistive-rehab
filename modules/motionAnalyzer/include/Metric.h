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
public:
    std::string name;
    struct RomParams
    {
        std::string tag_joint;
        std::string tag_plane;
        yarp::sig::Vector ref_dir;
        std::string ref_joint;
        double minv,maxv;
    } rom_params;
    Rom() {;}
    Rom(const std::string &type,const std::string &name, const RomParams &params);
    Rom(const Rom &r);
    Rom& operator = (const Rom &r);

    std::string getTagJoint() const { return rom_params.tag_joint; }
    std::string getTagPlane() const { return rom_params.tag_plane; }
    std::string getRefJoint() const { return rom_params.ref_joint; }
    yarp::sig::Vector getRefDir() const { return rom_params.ref_dir; }

    yarp::os::Property getParams() const override;
    void print(std::ostream &os=std::cout) const override;

};

class Step : public Metric
{
public:
    std::string name;
    struct StepParams
    {
        yarp::sig::Vector num;
        yarp::sig::Vector den;
        size_t median_filter_window;
        double thresh,step_window,time_window,minv,maxv;
    } step_params;

    Step() {;}
    Step(const std::string &type, const std::string &name, const StepParams &params);
    Step(const Step &r);
    Step& operator = (const Step &r);

    yarp::sig::Vector getNum() const { return step_params.num; }
    yarp::sig::Vector getDen() const { return step_params.den; }
    size_t getFilterWindow() const { return step_params.median_filter_window; }
    double getThresh() const { return step_params.thresh; }
    double getStepWindow() const { return step_params.step_window; }
    double getTimeWindow() const { return step_params.time_window; }

    yarp::os::Property getParams() const override;
    void print(std::ostream &os=std::cout) const override;

};

class EndPoint : public Metric
{
public:
    std::string name;
    struct EndPointParams
    {
        std::string tag_joint;
        std::string tag_plane;
        yarp::sig::Vector ref_dir;
        double minv,maxv;
        yarp::sig::Vector target;
    } ep_params;
    EndPoint() {;}
    EndPoint(const std::string &type_,const std::string &name_,const EndPointParams &params);
    EndPoint(const EndPoint &ep);
    EndPoint& operator = (const EndPoint &ep);

    std::string getTagJoint() const { return ep_params.tag_joint; }
    std::string getTagPlane() const { return ep_params.tag_plane; }
    yarp::sig::Vector getRefDir() const { return ep_params.ref_dir; }
    yarp::sig::Vector getTarget() const { return ep_params.target; }

    yarp::os::Property getParams() const override;
    void print(std::ostream &os=std::cout) const override;

};

#endif
