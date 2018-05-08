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
using namespace assistive_rehab;

const string Rom_Processor::motion_type = "ROM";

Processor* createProcessor(const string& motion_tag, const Metric* metric_)
{
    if(motion_tag.compare(Rom_Processor::motion_type) >= 0)
    {
        yInfo() << "Creating processor for" << Rom_Processor::motion_type << "\n";
        return new Rom_Processor(metric_);
    }
    else
        return 0;
}

/********************************************************/
Processor::Processor()
{
    deviation = 0.0;
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

void Processor::setInitialConf(const SkeletonWaist &skeleton_init_, const map<string, pair<string, double> > &keypoints2conf_,
                               SkeletonWaist &skeleton)
{
    skeleton_init.update(skeleton_init_.get_unordered());
    skeleton_init.setTag("init");
    keypoints2conf = keypoints2conf_;

    if(!skeleton.update_planes())
        yError() << "Not all planes are updated";

    coronal = skeleton.getCoronal();
    sagittal = skeleton.getSagittal();
    transverse = skeleton.getTransverse();
    Vector p = skeleton[KeyPointTag::shoulder_center]->getPoint();
    Matrix T1(4,4);
    T1.setSubcol(coronal,0,0);
    T1.setSubcol(sagittal,0,1);
    T1.setSubcol(transverse,0,2);
    T1.setSubcol(p,0,3);
    T1(3,3)=1.0;

    inv_reference_system = SE3inv(T1);

    //    skeleton_init.print();
}

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
//    curr_skeleton.print();

    //    skeleton_init.print();
//    cout << endl;


}

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
                deviation+= isDeviatingFromIntialPose(*curr_skeleton[i], *skeleton_init[i]);
                isDeviating = true;
//                yWarning() << curr_skeleton[i]->getTag() << "deviating from initial pose";
            }
        }
    }
    cout << "\n";

    return isDeviating;
}

bool Processor::isOutOfSphere(const KeyPoint& keypoint, const KeyPoint& keypoint_init)
{
    Vector curr_kp(keypoint.getPoint()), curr_kp_init(keypoint_init.getPoint());
    double deviation = norm(curr_kp-curr_kp_init);

    yInfo() << keypoint.getTag().c_str() << deviation;
    return (deviation > keypoints2conf[keypoint.getTag()].second);
}

double Processor::isDeviatingFromIntialPose(const KeyPoint& keypoint, const KeyPoint& keypoint_init)
{
    Vector curr_kp(keypoint.getPoint()), curr_kp_init(keypoint_init.getPoint());
    Vector k1=curr_kp;
    k1.push_back(1.0);
    Vector transformed_kp = inv_reference_system*k1;
    double dev = norm(transformed_kp.subVector(0,2)-curr_kp_init);

    yInfo() << keypoint.getTag().c_str()
            << dev
            << transformed_kp.subVector(0,2).toString()
            << curr_kp_init.toString();

    if(dev > keypoints2conf[keypoint.getTag()].second)
        return dev;
    else
        return 0.0;
}

/********************************************************/
Rom_Processor::Rom_Processor()
{

}

Rom_Processor::Rom_Processor(const Metric *rom_)
{
    rom = (Rom*)rom_;
}

//void Rom_Processor::setInitialConf(const SkeletonWaist &skeleton_init_, const map<string, pair<string, double> > &keypoints2conf_)
//{
//    skeleton_init.update(skeleton_init_.get_unordered());
//    keypoints2conf = keypoints2conf_;
//}

double Rom_Processor::computeMetric(Vector &v1, Vector &plane_normal_, Vector &ref_dir, double &score_exercise)
{
    double result;
    v1.resize(3);
    plane_normal_.resize(3);
    ref_dir.resize(3);

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
                plane_normal_ = cor;
                component_to_check = 0;
            }
            else if(rom->getTagPlane() == "sagittal")
            {
                Vector sag(3,0.0);
                sag[1]=1.0;
                plane_normal_ = sag;
                component_to_check = 1;
            }
            else if(rom->getTagPlane() == "transverse")
            {
                Vector trans(3,0.0);
                trans[2]=1.0;
                plane_normal_ = trans;
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
            if(abs(v1[component_to_check])>rom->getRangePlane())
            {
//                yInfo() << "out of the plane band" << v1[component_to_check];
                score_exercise = 0.4;
            }

            double dist = dot(v1,plane_normal_);
            v1 = v1-dist*plane_normal_;
            double n1 = norm(v1);
            if(n1 > 0.0)
                v1 /= n1;

            double n2 = norm(ref_dir);
            double dot_p = dot(v1,ref_dir);

            theta = acos(dot_p/(n1*n2));

            result = theta * (180/M_PI);
            prev_result = result;    
        }
        else
        {
            yError() << "The keypoint does not have a child ";
            result = 0.0;
            score_exercise = 0.0;
            v1.zero();
            plane_normal_.zero();
            ref_dir.zero();
        }
    }
    else
    { 
        result = prev_result;
        score_exercise = 0.0;
        v1.zero();
        plane_normal_.zero();
        ref_dir.zero();
    }


    return result;

}
