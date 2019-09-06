/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Metric.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include "Metric.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

namespace MetricType
{
const string rom="ROM";
const string end_point="EP";
const string step="step";
}

Metric::Metric()
{

}

Metric::~Metric()
{

}

/************************/
/*        ROM           */
/************************/
Rom::Rom(const string &type_, const string &name_, const string &tag_joint_,
         const string &tag_plane_, const Vector &ref_dir_, const string &ref_joint_,
         const double &minv_, const double &maxv_)
{
    type=type_;
    name=name_;
    tag_joint=tag_joint_;
    tag_plane=tag_plane_;
    ref_dir=ref_dir_;
    ref_joint=ref_joint_;
    minv=minv_;
    maxv=maxv_;
    properties.push_back("range");
}

Rom::Rom(const Rom &r)
{
    name=r.name;
    tag_joint=r.tag_joint;
    tag_plane=r.tag_plane;
    ref_dir=r.ref_dir;
    ref_joint=r.ref_joint;
    minv=r.minv;
    maxv=r.maxv;
    properties=r.properties;
}

Rom& Rom::operator = (const Rom &r)
{
    name=r.name;
    tag_joint=r.tag_joint;
    tag_plane=r.tag_plane;
    ref_dir=r.ref_dir;
    ref_joint=r.ref_joint;
    minv=r.minv;
    maxv=r.maxv;
    properties=r.properties;
    return *this;
}

void Rom::print(ostream &os) const
{
    os<<"\t"<< "type= "<<type<<endl<<"\t";
    os<<"name= "<<name<<endl<<"\t";
    os<<"tag_joint= "<<tag_joint<<endl<<"\t";
    os<<"tag_plane= "<<tag_plane<<endl<<"\t";
    os<<"ref_dir= "<<ref_dir.toString(3,1)<<endl<<"\t";
    os<<"ref_joint= "<<ref_joint<<endl<<"\t";
    os<<"min= "<<minv<<endl<<"\t";
    os<<"max= "<<maxv<<endl;
}

yarp::os::Property Rom::getParams() const
{
    Property params;
    params.put("type",type);
    params.put("name",name);
    params.put("tag_joint",tag_joint);
    params.put("tag_plane",tag_plane);

    Bottle bRefDir;
    bRefDir.addList().read(ref_dir);
    params.put("ref_dir",bRefDir.get(0));

    params.put("ref_joint",ref_joint);
    params.put("min",minv);
    params.put("max",maxv);
    return params;
}

/************************/
/*        Step          */
/************************/
Step::Step(const string &type_, const string &name_, const yarp::sig::Vector &num_,
           const yarp::sig::Vector &den_, const double &minv_, const double &maxv_)
{
    type=type_;
    name=name_;
    num=num_;
    den=den_;
    minv=minv_;
    maxv=maxv_;

    properties.push_back("step_length");
    properties.push_back("step_width");
    properties.push_back("cadence");
    properties.push_back("speed");
}

Step::Step(const Step &r)
{
    type=r.type;
    name=r.name;
    num=r.num;
    den=r.den;
    minv=r.minv;
    maxv=r.maxv;
    properties=r.properties;
}

Step& Step::operator = (const Step &r)
{
    type=r.type;
    name=r.name;
    num=r.num;
    den=r.den;
    minv=r.minv;
    maxv=r.maxv;
    return *this;
}

void Step::print(ostream &os) const
{
    os<<"\t"<<"type= "<<type<<endl<<"\t";
    os<<"name= "<<name<<endl<<"\t";
    os<<"min= "<<minv<<endl<<"\t";
    os<<"max= "<<maxv<<endl;
}

yarp::os::Property Step::getParams() const
{
    Property params;
    params.put("type",type);
    params.put("name",name);

    Bottle bNum;
    bNum.addList().read(num);
    params.put("num",bNum.get(0));

    Bottle bDen;
    bDen.addList().read(den);
    params.put("den",bDen.get(0));

    params.put("min",minv);
    params.put("max",maxv);
    return params;
}

/************************/
/*      END POINT       */
/************************/
EndPoint::EndPoint(const string &type_, const string &name_, const string &tag_joint_,
                   const string &tag_plane_, const Vector &ref_dir_, const double &minv_,
                   const double &maxv_, const Vector &target_)
{
    type=type_;
    name=name_;
    tag_joint=tag_joint_;
    tag_plane=tag_plane_;
    ref_dir=ref_dir_;
    minv=minv_;
    maxv=maxv_;
    target=target_;

    properties.push_back("trajectory");
    properties.push_back("speed");
    properties.push_back("smoothness");
}

EndPoint::EndPoint(const EndPoint &ep)
{
    type=ep.type;
    name=ep.name;
    tag_joint=ep.tag_joint;
    tag_plane=ep.tag_plane;
    ref_dir=ep.ref_dir;
    minv=ep.minv;
    maxv=ep.maxv;
    target=ep.target;
    properties=ep.properties;
}

EndPoint& EndPoint::operator = (const EndPoint &ep)
{
    type=ep.type;
    name=ep.name;
    tag_joint=ep.tag_joint;
    tag_plane=ep.tag_plane;
    ref_dir=ep.ref_dir;
    minv=ep.minv;
    maxv=ep.maxv;
    target=ep.target;
    properties=ep.properties;
    return *this;
}

void EndPoint::print(ostream &os) const
{
    os<<"\t"<<"type= "<<type<<endl<<"\t";
    os<<"name= "<<name<<endl<<"\t";
    os<<"tag_joint= "<<tag_joint<<endl<<"\t";
    os<<"tag_plane= "<<tag_plane<<endl<<"\t";
    os<<"ref_dir= "<<ref_dir.toString(3,1)<<endl<<"\t";
    os<<"min= "<<minv<<endl<<"\t";
    os<<"max= "<<maxv<<endl<<"\t";
    os<<"target= "<<target.toString(3,1)<<endl;
}

yarp::os::Property EndPoint::getParams() const
{
    Property params;
    params.put("type",type);
    params.put("name",name);
    params.put("tag_joint",tag_joint);
    params.put("tag_plane",tag_plane);

    Bottle bRefDir;
    bRefDir.addList().read(ref_dir);
    params.put("ref_dir",bRefDir.get(0));

    params.put("min",minv);
    params.put("max",maxv);

    Bottle bTarget;
    bTarget.addList().read(target);
    params.put("target",bTarget.get(0));

    return params;
}
