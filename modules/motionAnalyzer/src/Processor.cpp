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

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::os;
using namespace iCub::ctrl;
using namespace assistive_rehab;

const string Rom_Processor::metric_tag = "ROM";
const string EndPoint_Processor::metric_tag = "EP";

/****************************************/
/*             PROCESSOR                */
/****************************************/
Processor* createProcessor(const string& motion_tag, const Metric* metric_)
{
    if(motion_tag.find(Rom_Processor::metric_tag) != std::string::npos)
    {
        yInfo() << "Creating processor for" << Rom_Processor::metric_tag << "\n";
        return new Rom_Processor(metric_);
    }
    else if(motion_tag.find(EndPoint_Processor::metric_tag) != std::string::npos)
    {
        yInfo() << "Creating processor for" << EndPoint_Processor::metric_tag << "\n";
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
void Processor::setInitialConf(SkeletonWaist &skeleton_, Matrix &T)
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
}

/****************************************************************/
void Processor::update(SkeletonWaist &curr_skeleton_)
{
    curr_skeleton.update(curr_skeleton_.toProperty());
    curr_skeleton.normalize();
}

/****************************************/
/*            ROM PROCESSOR             */
/****************************************/
Rom_Processor::Rom_Processor()
{

}

/********************************************************/
Rom_Processor::Rom_Processor(const Metric *rom_)
{
    rom = (Rom*)rom_;
    prev_result = 0.0;
}

/********************************************************/
double Rom_Processor::computeMetric()
{
    double result;
    Vector v1(3,0.0);
    Vector ref_dir(3,0.0);

    //get reference keypoint from skeleton
    string tag_joint = rom->getTagJoint();

    if(curr_skeleton[tag_joint]->isUpdated() && curr_skeleton[tag_joint]->getChild(0)->isUpdated())
    {
        Vector kp_ref = curr_skeleton[tag_joint]->getPoint();

        double theta;
        if(curr_skeleton[tag_joint]->getNumChild())
        {
            Vector kp_child = curr_skeleton[tag_joint]->getChild(0)->getPoint();

            if(rom->getTagPlane() == "coronal")
            {
                Vector cor(3,0.0);
                cor[0]=1.0;
                plane_normal = cor;
            }
            else if(rom->getTagPlane() == "sagittal")
            {
                Vector sag(3,0.0);
                sag[1]=1.0;
                plane_normal = sag;
            }
            else if(rom->getTagPlane() == "transverse")
            {
                Vector trans(3,0.0);
                trans[2]=1.0;
                plane_normal = trans;
            }

            ref_dir = rom->getRefDir();

            Vector k1=kp_ref;
            k1.push_back(1.0);
            Vector k2=kp_child;
            k2.push_back(1.0);
            Vector transformed_kp_ref = inv_reference_system*k1;
            Vector transformed_kp_child = inv_reference_system*k2;

            v1 = transformed_kp_child.subVector(0,2)-transformed_kp_ref.subVector(0,2);
            double dist = dot(v1,plane_normal);
            v1 = v1-dist*plane_normal;
            double n1 = norm(v1);
            if(n1 > 0.0)
                v1 /= n1;

            double n2 = norm(ref_dir);
            double dot_p = dot(v1,ref_dir);

            theta = acos(dot_p/n2); 
            result = theta * (180/M_PI);           
            prev_result = result;
        }
        else
        {
            yError() << "The keypoint does not have a child ";
            result = 0.0;
        }
    }
    else
    { 
        result = prev_result;
    }

    return result;

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
    ep = (EndPoint*)ep_;
    linEst = new AWLinEstimator(16,0.5);
    jerkEst = new JerkEstimator(16,0.1);
    ideal_traj = 0.0;
    prev_est_traj = 0.0;
    prev_ideal_traj = 0.0;
    prev_vel = 0.0;
    prev_smoothness = 0.0;
}

/********************************************************/
EndPoint_Processor::~EndPoint_Processor()
{
    delete linEst;
    delete jerkEst;
}

/********************************************************/
double EndPoint_Processor::computeMetric()
{
    if(ep->getTagPlane() == "coronal")
        plane_normal[0]=1.0;
    else if(ep->getTagPlane() == "sagittal")
        plane_normal[1]=1.0;
    else if(ep->getTagPlane() == "transverse")
        plane_normal[2]=1.0;

    double est_traj,vel,smoothness;
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

        Vector t = ep->getTarget();
        t.push_back(1.0);
        Vector transformed_t = inv_reference_system*t;
        Vector dt = transformed_t.subVector(0,2)-transformed_ref.subVector(0,2);
        ideal_traj = getTrajectory(dt);

        //we compute velocity and smoothness of the end-point
        AWPolyElement el(transformed_v,Time::now());
        vel = norm(linEst->estimate(el));
        smoothness = norm(jerkEst->estimate(el));

        prev_est_traj = est_traj;
        prev_ideal_traj = ideal_traj;
        prev_vel = vel;
        prev_smoothness = smoothness;
    }
    else
    {
        est_traj = prev_est_traj;
        ideal_traj = prev_ideal_traj;
        vel = prev_vel;
        smoothness = prev_smoothness;
    }

    //estimate vel and smoothness
    ep->setVel(vel);
    ep->setSmoothness(smoothness);

    return vel;
}

/********************************************************/
double EndPoint_Processor::getTrajectory(const Vector &v)
{
    double dist = dot(v,plane_normal);
    Vector proj = v-dist;
    double traj = norm(proj);

    return traj;
}
