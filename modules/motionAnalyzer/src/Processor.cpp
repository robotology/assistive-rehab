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
using namespace yarp::math;
using namespace yarp::os;
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
    deviation = 0.0;
    score_exercise = 0.0;
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
void Processor::setInitialConf(SkeletonWaist* skeleton_init_, const map<string, pair<string, double> > &keypoints2conf_,
                               SkeletonWaist &skeleton_)
{
    skeleton_init = skeleton_init_;
    keypoints2conf = keypoints2conf_;
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

 //   skeleton_init.print();
}

/********************************************************/
bool Processor::isStatic(const KeyPoint& keypoint)
{
    if(keypoints2conf[keypoint.getTag()].first == "static")
        return true;
    else
        return false;
}

/****************************************************************/
void Processor::update(SkeletonWaist &curr_skeleton_)
{
    curr_skeleton.update(curr_skeleton_.toProperty());
    curr_skeleton.normalize();
}

/********************************************************/
bool Processor::isDeviatingFromIntialPose()
{
    bool isDeviating = false;
    deviation = 0.0;

    for(unsigned int i=0; i<curr_skeleton.getNumKeyPoints(); i++)
    {
        if(curr_skeleton[i]->isUpdated() && curr_skeleton[i]->getTag() != KeyPointTag::hip_center)
        {
            if(isStatic(*curr_skeleton[i]))
            {
                deviation+= isDeviatingFromIntialPose(*curr_skeleton[i], *(*skeleton_init)[i]);
                isDeviating = true;
//                yWarning() << curr_skeleton[i]->getTag() << "deviating from initial pose";
            }
        }
    }
//    cout << "\n";

    return isDeviating;
}

/********************************************************/
double Processor::isDeviatingFromIntialPose(const KeyPoint& keypoint, const KeyPoint& keypoint_init)
{
    Vector k1=keypoint.getPoint();
    k1.push_back(1.0);
    Vector transformed_kp = (inv_reference_system*k1).subVector(0,2);
    double dev = norm(transformed_kp-keypoint_init.getPoint());

/*    yInfo() << keypoint.getTag()
            << dev
            << transformed_kp.toString(3,3)
            << keypoint_init.getPoint().toString(3,3); */

    if(dev > keypoints2conf[keypoint.getTag()].second)
        return dev;
    else
        return 0.0;
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
    prev_score = 0.0;
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

            int component_to_check;
            if(rom->getTagPlane() == "coronal")
            {
                Vector cor(3,0.0);
                cor[0]=1.0;
                plane_normal = cor;
                component_to_check = 0;
            }
            else if(rom->getTagPlane() == "sagittal")
            {
                Vector sag(3,0.0);
                sag[1]=1.0;
                plane_normal = sag;
                component_to_check = 1;
            }
            else if(rom->getTagPlane() == "transverse")
            {
                Vector trans(3,0.0);
                trans[2]=1.0;
                plane_normal = trans;
                component_to_check = 2;
            }

            ref_dir = rom->getRefDir();

            Vector k1=kp_ref;
            k1.push_back(1.0);
            Vector k2=kp_child;
            k2.push_back(1.0);
            Vector transformed_kp_ref = inv_reference_system*k1;
            Vector transformed_kp_child = inv_reference_system*k2;

            v1 = transformed_kp_child.subVector(0,2)-transformed_kp_ref.subVector(0,2);

            score_exercise = 0.7;
            //if(abs(v1[component_to_check])>rom->getRangePlane())
            //{
            //    yInfo() << "out of the plane band" << v1[component_to_check];
            //    score_exercise = 0.4;
            //}

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
            prev_score = score_exercise;    
        }
        else
        {
            yError() << "The keypoint does not have a child ";
            result = 0.0;
            score_exercise = 0.0;
        }
    }
    else
    { 
        result = prev_result;
        score_exercise = prev_score;
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
    linEst = new AWLinEstimator(16,1.0);
    polyEst = new AWThirdEstimator(16,1.0);
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
    delete polyEst;
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

    double est_traj;
    string tag_joint = ep->getTagJoint();
    if(curr_skeleton[tag_joint]->isUpdated())
    {
        Vector ref = first_skeleton[tag_joint]->getParent(0)->getParent(0)->getPoint();
        ref.push_back(1.0);
        Vector v = curr_skeleton[tag_joint]->getPoint();
        v.push_back(1.0);
        est_traj = getTrajectory(v,ref);

        Vector t = ep->getTarget();
        t.push_back(1.0);
        ideal_traj = getTrajectory(t,ref);

        prev_est_traj = est_traj;
        prev_ideal_traj = ideal_traj;
    }
    else
    {
        est_traj = prev_est_traj;
        ideal_traj = prev_ideal_traj;
    }

    //estimate vel and smoothness
    estimate();

    return est_traj;
}

/********************************************************/
void EndPoint_Processor::estimate()
{
    double vel, smoothness;
    string tag_joint = ep->getTagJoint();
    if(curr_skeleton[tag_joint]->isUpdated())
    {
        Vector kp = curr_skeleton[tag_joint]->getPoint();
        kp.push_back(1.0);
        Vector transformed_kp = inv_reference_system*kp;
        AWPolyElement el(transformed_kp,Time::now());
        vel = norm(linEst->estimate(el));
        smoothness = norm(polyEst->estimate(el));
        prev_vel = vel;
        prev_smoothness = smoothness;
    }
    else
    {
        vel = prev_vel;
        smoothness = prev_smoothness;
    }

    ep->setVel(vel);
    ep->setSmoothness(smoothness);
}

/********************************************************/
double EndPoint_Processor::getTrajectory(const Vector &k, const Vector &kref)
{
    Vector transformed_k = inv_reference_system*k;
    Vector transformed_kref = inv_reference_system*kref;
    Vector v = transformed_k.subVector(0,2)-transformed_kref.subVector(0,2);

    double dist = dot(v,plane_normal);
    v = v-dist;
    double traj = norm(v);

    return traj;
}
