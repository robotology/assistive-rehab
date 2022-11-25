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
Rom::Rom(const string &type, const string &name, const RomParams &params)
{
    this->type=type;
    this->name=name;
    this->rom_params.tag_joint=params.tag_joint;
    this->rom_params.tag_plane=params.tag_plane;
    this->rom_params.ref_dir=params.ref_dir;
    this->rom_params.ref_joint=params.ref_joint;
    this->rom_params.minv=params.minv;
    this->rom_params.maxv=params.maxv;
    properties.push_back("range");
}

Rom::Rom(const Rom &r)
{
    this->name=r.name;
    this->rom_params.tag_joint=r.rom_params.tag_joint;
    this->rom_params.tag_plane=r.rom_params.tag_plane;
    this->rom_params.ref_dir=r.rom_params.ref_dir;
    this->rom_params.ref_joint=r.rom_params.ref_joint;
    this->rom_params.minv=r.rom_params.minv;
    this->rom_params.maxv=r.rom_params.maxv;
    this->properties=r.properties;
}

Rom& Rom::operator = (const Rom &r)
{
    this->name=r.name;
    this->rom_params.tag_joint=r.rom_params.tag_joint;
    this->rom_params.tag_plane=r.rom_params.tag_plane;
    this->rom_params.ref_dir=r.rom_params.ref_dir;
    this->rom_params.ref_joint=r.rom_params.ref_joint;
    this->rom_params.minv=r.rom_params.minv;
    this->rom_params.maxv=r.rom_params.maxv;
    this->properties=r.properties;
    return *this;
}

void Rom::print(ostream &os) const
{
    os<<"\t"<< "type= "<<type<<endl<<"\t";
    os<<"name= "<<name<<endl<<"\t";
    os<<"tag_joint= "<<rom_params.tag_joint<<endl<<"\t";
    os<<"tag_plane= "<<rom_params.tag_plane<<endl<<"\t";
    os<<"ref_dir= "<<rom_params.ref_dir.toString(3,1)<<endl<<"\t";
    os<<"ref_joint= "<<rom_params.ref_joint<<endl<<"\t";
    os<<"min= "<<rom_params.minv<<endl<<"\t";
    os<<"max= "<<rom_params.maxv<<endl;
}

yarp::os::Property Rom::getParams() const
{
    Property params;
    params.put("type",type);
    params.put("name",name);
    params.put("tag_joint",rom_params.tag_joint);
    params.put("tag_plane",rom_params.tag_plane);

    Bottle bRefDir;

    Bottle bb;
    bb = bRefDir.addList();

    yDebug() <<rom_params.ref_dir.toString(3,1);

    bb.read(rom_params.ref_dir);
    params.put("ref_dir",bRefDir.get(0));

    params.put("ref_joint",rom_params.ref_joint);
    params.put("min",rom_params.minv);
    params.put("max",rom_params.maxv);
    return params;
}

/************************/
/*        Step          */
/************************/
Step::Step(const string &type, const string &name, const StepParams &params)
{
    this->type=type;
    this->name=name;
    this->step_params=params;
    properties.push_back("step_length");
    properties.push_back("step_width");
    properties.push_back("cadence");
    properties.push_back("speed");
    properties.push_back("num_steps");
    properties.push_back("step_distance");
}

Step::Step(const Step &r)
{
    this->type=r.type;
    this->name=r.name;
    this->step_params.num=r.step_params.num;
    this->step_params.den=r.step_params.den;
    this->step_params.median_filter_window=r.step_params.median_filter_window;
    this->step_params.thresh=r.step_params.thresh;
    this->step_params.step_window=r.step_params.step_window;
    this->step_params.time_window=r.step_params.time_window;
    this->step_params.minv=r.step_params.minv;
    this->step_params.maxv=r.step_params.maxv;
    this->properties=r.properties;
}

Step& Step::operator = (const Step &r)
{
    this->type=r.type;
    this->name=r.name;
    this->step_params.num=r.step_params.num;
    this->step_params.den=r.step_params.den;
    this->step_params.thresh=r.step_params.thresh;
    this->step_params.step_window=r.step_params.step_window;
    this->step_params.time_window=r.step_params.time_window;
    this->step_params.minv=r.step_params.minv;
    this->step_params.maxv=r.step_params.maxv;
    return *this;
}

void Step::print(ostream &os) const
{
    os<<"\t"<<"type= "<<type<<endl<<"\t";
    os<<"name= "<<name<<endl<<"\t";
    os<<"thresh= "<<step_params.thresh<<endl<<"\t";
    os<<"step window= "<<step_params.step_window<<endl<<"\t";
    os<<"time window= "<<step_params.time_window<<endl<<"\t";
    os<<"min= "<<step_params.minv<<endl<<"\t";
    os<<"max= "<<step_params.maxv<<endl;
}

yarp::os::Property Step::getParams() const
{
    Property params;
    params.put("type",type);
    params.put("name",name);

    Bottle bNum;
    bNum.addList().read(step_params.num);
    params.put("num",bNum.get(0));

    Bottle bDen;
    bDen.addList().read(step_params.den);
    params.put("den",bDen.get(0));

    params.put("min",step_params.minv);
    params.put("max",step_params.maxv);
    params.put("step_thresh",step_params.thresh);
    params.put("step_window",step_params.step_window);
    params.put("time_window",step_params.time_window);
    return params;
}

/************************/
/*      END POINT       */
/************************/
EndPoint::EndPoint(const string &type, const string &name, const EndPointParams &params)
{
    this->type=type;
    this->name=name;
    this->ep_params.tag_joint=params.tag_joint;
    this->ep_params.tag_plane=params.tag_plane;
    this->ep_params.ref_dir=params.ref_dir;
    this->ep_params.minv=params.minv;
    this->ep_params.maxv=params.maxv;
    this->ep_params.target=params.target;

    properties.push_back("trajectory");
    properties.push_back("speed");
    properties.push_back("smoothness");
}

EndPoint::EndPoint(const EndPoint &ep)
{
    this->type=ep.type;
    this->name=ep.name;
    this->ep_params.tag_joint=ep.ep_params.tag_joint;
    this->ep_params.tag_plane=ep.ep_params.tag_plane;
    this->ep_params.ref_dir=ep.ep_params.ref_dir;
    this->ep_params.minv=ep.ep_params.minv;
    this->ep_params.maxv=ep.ep_params.maxv;
    this->ep_params.target=ep.ep_params.target;
    this->properties=ep.properties;
}

EndPoint& EndPoint::operator = (const EndPoint &ep)
{
    this->type=ep.type;
    this->name=ep.name;
    this->ep_params.tag_joint=ep.ep_params.tag_joint;
    this->ep_params.tag_plane=ep.ep_params.tag_plane;
    this->ep_params.ref_dir=ep.ep_params.ref_dir;
    this->ep_params.minv=ep.ep_params.minv;
    this->ep_params.maxv=ep.ep_params.maxv;
    this->ep_params.target=ep.ep_params.target;
    this->properties=ep.properties;
    return *this;
}

void EndPoint::print(ostream &os) const
{
    os<<"\t"<<"type= "<<type<<endl<<"\t";
    os<<"name= "<<name<<endl<<"\t";
    os<<"tag_joint= "<<ep_params.tag_joint<<endl<<"\t";
    os<<"tag_plane= "<<ep_params.tag_plane<<endl<<"\t";
    os<<"ref_dir= "<<ep_params.ref_dir.toString(3,1)<<endl<<"\t";
    os<<"min= "<<ep_params.minv<<endl<<"\t";
    os<<"max= "<<ep_params.maxv<<endl<<"\t";
    os<<"target= "<<ep_params.target.toString(3,1)<<endl;
}

yarp::os::Property EndPoint::getParams() const
{
    Property params;
    params.put("type",type);
    params.put("name",name);
    params.put("tag_joint",ep_params.tag_joint);
    params.put("tag_plane",ep_params.tag_plane);

    Bottle bRefDir;
    bRefDir.addList().read(ep_params.ref_dir);
    params.put("ref_dir",bRefDir.get(0));

    params.put("min",ep_params.minv);
    params.put("max",ep_params.maxv);

    Bottle bTarget;
    bTarget.addList().read(ep_params.target);
    params.put("target",bTarget.get(0));

    return params;
}
