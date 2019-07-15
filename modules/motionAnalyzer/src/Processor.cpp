/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Processor.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include "Processor.h"
#include <iomanip>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace assistive_rehab;

/****************************************/
/*             PROCESSOR                */
/****************************************/
Processor *createProcessor(const string& metric_type, const Metric* metric_)
{
    if(metric_type.find(MetricType::rom)!=std::string::npos)
    {
        yInfo() << "Creating processor for" << MetricType::rom << "\n";
        return new Rom_Processor(metric_);
    }
    else if(metric_type.find(MetricType::step)!=std::string::npos)
    {
        yInfo() << "Creating processor for" << MetricType::step << "\n";
        return new Step_Processor(metric_);
    }
    else if(metric_type.find(MetricType::end_point)!=std::string::npos)
    {
        yInfo() << "Creating processor for" << MetricType::end_point << "\n";
        return new EndPoint_Processor(metric_);
    }
    else
        return 0;
}

/********************************************************/
Processor::Processor()
{
    plane_normal.resize(3);
    invT.resize(4,4);
    invT.zero();
}

void print(const Matrix& m)
{
    for(int i=0;i<m.rows();i++)
    {
        for(int j=0;j<m.cols();j++)
        {
            cout << m[i][j] << " ";
        }
        cout << "\n";
    }
}

/********************************************************/
void Processor::setInitialConf(SkeletonStd &skeleton_, Matrix &T)
{
    first_skeleton.update(skeleton_.toProperty());
    first_skeleton.normalize();

    if(!first_skeleton.update_planes())
        yError() << "Not all planes are updated";

    coronal = first_skeleton.getCoronal();
    sagittal = first_skeleton.getSagittal();
    transverse = first_skeleton.getTransverse();
    Vector p = first_skeleton[KeyPointTag::shoulder_center]->getPoint();
    Matrix T1(4,4);
    T1.setSubcol(coronal,0,0);
    T1.setSubcol(sagittal,0,1);
    T1.setSubcol(transverse,0,2);
    T1.setSubcol(p,0,3);
    T1(3,3)=1.0;
    inv_reference_system = SE3inv(T1);
    T = inv_reference_system;

    t0=Time::now();
}

/****************************************************************/
void Processor::update(SkeletonStd &curr_skeleton_)
{
    curr_skeleton.update(curr_skeleton_.toProperty());
}

/****************************************************************/
Vector Processor::projectOnPlane(const Vector &v,const Vector &plane)
{
    double dist=dot(v,plane);
    return v-dist*plane;
}

/****************************************************************/
Vector Processor::getKeypointSkel(const string &tag)
{
    Vector k=curr_skeleton[tag]->getPoint();
    k.push_back(1.0);
    return (inv_reference_system*k).subVector(0,2);
}

/****************************************/
/*            ROM PROCESSOR             */
/****************************************/
Rom_Processor::Rom_Processor()
{

}

/********************************************************/
Rom_Processor::Rom_Processor(const Metric* rom_)
{
    rom=(Rom*)rom_;
    prev_range=0.0;
}

/********************************************************/
void Rom_Processor::estimate()
{
    Vector v1(3,0.0);
    Vector ref_dir(3,0.0);

    //get reference keypoint from skeleton
    curr_skeleton.normalize();
    string tag_joint=rom->getTagJoint();
    if(curr_skeleton[tag_joint]->isUpdated() && curr_skeleton[tag_joint]->getChild(0)->isUpdated())
    {
        double theta;
        if(curr_skeleton[tag_joint]->getNumChild())
        {
            if(rom->getTagPlane()=="coronal")
            {
                Vector cor(3,0.0);
                cor[0]=1.0;
                plane_normal=cor;
            }
            else if(rom->getTagPlane()=="sagittal")
            {
                Vector sag(3,0.0);
                sag[1]=1.0;
                plane_normal=sag;
            }
            else if(rom->getTagPlane()=="transverse")
            {
                Vector trans(3,0.0);
                trans[2]=1.0;
                plane_normal=trans;
            }
            Vector k1=getKeypointSkel(tag_joint);
            Vector k2=getKeypointSkel(curr_skeleton[tag_joint]->getChild(0)->getTag());
            v1=k2-k1;
            v1=projectOnPlane(v1,plane_normal);
            double n1=norm(v1);
            if(n1>0.0)
                v1/=n1;
            if(!rom->getRefJoint().empty())
            {
                Vector k_dir=getKeypointSkel(rom->getRefJoint());
                ref_dir=k1-k_dir;
            }
            else
            {
                ref_dir=rom->getRefDir();
            }
            double n2=norm(ref_dir);
            double dot_p=dot(v1,ref_dir);
            theta=acos(dot_p/n2);
            range=theta*(180/M_PI);
            prev_range=range;
        }
        else
        {
            yError() << "The keypoint does not have a child ";
            range=0.0;
        }
    }
    else
    {
        range=prev_range;
    }
}

/********************************************************/
yarp::os::Property Rom_Processor::getResult()
{
    Property result;
    vector<string> props=rom->getProperties();
    result.put(props[0],range);
    return result;
}

/****************************************/
/*            STEP PROCESSOR            */
/****************************************/
Step_Processor::Step_Processor()
{

}

/********************************************************/
Step_Processor::Step_Processor(const Metric* step_)
{
    step=(Step*)step_;
    filter_dist=new Filter(step->getNum(),step->getDen());
    filter_width=new Filter(step->getNum(),step->getDen());

    prev_steplen=0.0;
    prev_stepwidth=0.0;
    prev_cadence=0.0;
    prev_speed=0.0;
}

/********************************************************/
void Step_Processor::estimate()
{
    if(curr_skeleton[KeyPointTag::ankle_left]->isUpdated() &&
            curr_skeleton[KeyPointTag::ankle_right]->isUpdated())
    {
        Vector k1=getKeypointSkel(KeyPointTag::ankle_left);
        Vector k2=getKeypointSkel(KeyPointTag::ankle_right);
        Vector v=k1-k2;
        double dist=norm(v);
        feetdist.push_back(dist);

        Vector cor_plane=curr_skeleton.getCoronal();
        Vector v_stepwidth=projectOnPlane(v,cor_plane);
        feetwidth.push_back(norm(v_stepwidth));

        estimateSpatialParams(feetdist,feetwidth);
        cadence=estimateCadence();
        speed=estimateSpeed();

        prev_steplen=steplen;
        prev_stepwidth=stepwidth;
        prev_cadence=cadence;
        prev_speed=speed;
    }
    else
    {
        steplen=prev_steplen;
        stepwidth=prev_stepwidth;
        cadence=prev_cadence;
        speed=prev_speed;
    }
//    yInfo()<<"Step parameters:"<<Time::now()-t0<<steplen<<stepwidth<<cadence<<speed<<numsteps;
}

/********************************************************/
Property Step_Processor::getResult()
{
    Property result;
    vector<string> props=step->getProperties();
    result.put(props[0],steplen);
    result.put(props[1],stepwidth);
    result.put(props[2],cadence);
    result.put(props[3],speed);
    return result;
}

/********************************************************/
void Step_Processor::estimateSpatialParams(const Vector &dist,const Vector &width)
{
    Vector filt_dist(dist.size()),filt_width(width.size());
    for(int i=0; i<dist.size(); i++)
    {
        Vector u1(1),u2(1);
        u1[0]=dist[i];
        u2[0]=width[i];
        Vector output1=filter_dist->filt(u1);
        Vector output2=filter_width->filt(u2);
        filt_dist[i]=output1[0];
        filt_width[i]=output2[0];
    }

    pair<vector<double>,vector<int>> step_params;
    step_params=findPeaks(filt_dist);
    vector<double> stepvec=step_params.first;
    vector<int> strikes=step_params.second;

    steplen=stepvec.size()>0 ? stepvec.back() : 0.0;
    int laststrike=strikes.size()>0 ? strikes.back() : 0.0;
    stepwidth=feetwidth[laststrike];
    numsteps=(int)strikes.size();
}

/********************************************************/
double Step_Processor::estimateCadence()
{
    return numsteps/(Time::now()-t0);
}

/********************************************************/
double Step_Processor::estimateSpeed()
{
    return steplen*cadence;
}

/********************************************************/
pair<vector<double>,vector<int>> Step_Processor::findPeaks(const Vector& d)
{
    vector<double> val;
    vector<int> idx;
    for(int i=0; i<d.size(); i++)
    {
        if(i==0 && d[i]>=d[i+1])
        {
            val.push_back(d[i]);
            idx.push_back(i);
        }
        else if(i==d.size() && d[i]>=d[i-1])
        {
            val.push_back(d[i]);
            idx.push_back(i);
        }
        else if(d[i]>=d[i-1] && d[i]>=d[i+1])
        {
            val.push_back(d[i]);
            idx.push_back(i);
        }
    }

    pair<vector<double>,vector<int>> step_params;
    step_params=make_pair(val,idx);
    return step_params;
}

/********************************************************/
Step_Processor::~Step_Processor()
{
    delete filter_dist;
    delete filter_width;
}

/****************************************/
/*        END POINT PROCESSOR           */
/****************************************/
EndPoint_Processor::EndPoint_Processor()
{

}

/********************************************************/
EndPoint_Processor::EndPoint_Processor(const Metric *ep_)
{
    ep=(EndPoint*)ep_;
    linEst=new AWLinEstimator(16,0.5);
    jerkEst=new JerkEstimator(16,0.1);
    ideal_traj=0.0;
    prev_est_traj=0.0;
    prev_ideal_traj=0.0;
    prev_vel=0.0;
    prev_smoothness=0.0;
}

/********************************************************/
EndPoint_Processor::~EndPoint_Processor()
{
    delete linEst;
    delete jerkEst;
}

/********************************************************/
void EndPoint_Processor::estimate()
{
    if(ep->getTagPlane() == "coronal")
        plane_normal[0]=1.0;
    else if(ep->getTagPlane() == "sagittal")
        plane_normal[1]=1.0;
    else if(ep->getTagPlane() == "transverse")
        plane_normal[2]=1.0;

    curr_skeleton.normalize();
    string tag_joint = ep->getTagJoint();
    if(curr_skeleton[tag_joint]->isUpdated())
    {
        //we compute ideal and current trajectory wrt left/right shoulder
        Vector ref = first_skeleton[tag_joint]->getParent(0)->getParent(0)->getPoint();
        ref.push_back(1.0);
        Vector v = curr_skeleton[tag_joint]->getPoint();
        v.push_back(1.0);
        Vector transformed_v = inv_reference_system*v;
        Vector transformed_ref = inv_reference_system*ref;
        Vector dv = transformed_v.subVector(0,2)-transformed_ref.subVector(0,2);
        est_traj = getTrajectory(dv);

        yDebug() << first_skeleton[tag_joint]->getParent(0)->getParent(0)->getTag() << ref.toString();
        yDebug() << tag_joint << v.toString();
        cout << endl;

        Vector t=ep->getTarget();
        t.push_back(1.0);
        Vector transformed_t = inv_reference_system*t;
        Vector dt = transformed_t.subVector(0,2)-transformed_ref.subVector(0,2);
        ideal_traj = getTrajectory(dt);

        //we compute velocity and smoothness of the end-point
        AWPolyElement el(transformed_v,Time::now());
        vel=norm(linEst->estimate(el));
        smoothness = norm(jerkEst->estimate(el));

        prev_est_traj=est_traj;
        prev_ideal_traj=ideal_traj;
        prev_vel=vel;
        prev_smoothness=smoothness;
    }
    else
    {
        est_traj=prev_est_traj;
        ideal_traj=prev_ideal_traj;
        vel=prev_vel;
        smoothness=prev_smoothness;
    }

}

/********************************************************/
Property EndPoint_Processor::getResult()
{
    Property result;
    vector<string> props=ep->getProperties();
    result.put(props[0],est_traj);
    result.put(props[1],vel);
    result.put(props[2],smoothness);
    return result;
}

/********************************************************/
double EndPoint_Processor::getTrajectory(const Vector &v)
{
    double dist=dot(v,plane_normal);
    Vector proj=v-dist;
    double traj=norm(proj);
    return traj;
}
