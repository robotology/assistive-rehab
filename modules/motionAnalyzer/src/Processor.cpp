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
    Matrix T1(3,4);
    T1.setCol(0,coronal);
    T1.setCol(1,sagittal);
    T1.setCol(2,transverse);
    T1.setCol(3,p);

    Matrix T2(4,4);
    T2.setSubmatrix(T1,0,0);
    Vector v(4,0.0);
    v[3]=1.0;
    T2.setRow(3,v);

    inv_reference_system = SE3inv(T2);

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

    Vector xyz=curr_skeleton[KeyPointTag::shoulder_center]->getPoint();
    Vector xyz_inv=invT.subcol(0,3,3);
    Vector rot(4,0.0);
    Matrix T=axis2dcm(rot);
    T(0,3)=xyz[0];
    T(1,3)=xyz[1];
    T(2,3)=xyz[2];
    invT=SE3inv(T);

    xyz+=xyz_inv;

    T(0,3)=xyz[0];
    T(1,3)=xyz[1];
    T(2,3)=xyz[2];

    skeleton_init.setTransformation(T);
    skeleton_init.update();
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
//    Vector curr_pose, initial_pose;

//    //if the current keypoint has child and parent
//    if(keypoint.getNumChild() && keypoint.getNumParent())
//    {
//        Vector curr_kp_parent = keypoint.getParent(0)->getPoint();
//        Vector curr_kp_child = keypoint.getChild(0)->getPoint();

//        Vector curr_kp_parent_init = keypoint_init.getParent(0)->getPoint();
//        Vector curr_kp_child_init = keypoint_init.getChild(0)->getPoint();

//        curr_pose = curr_kp + curr_kp_parent + curr_kp_child;
//        initial_pose = curr_kp_init + curr_kp_parent_init + curr_kp_child_init;
//    }
//    else if(keypoint.getNumParent()) //if the current keypoint has parent and not child
//    {
//        Vector curr_kp_parent = keypoint.getParent(0)->getPoint();
//        Vector curr_kp_parent_init = keypoint_init.getParent(0)->getPoint();

//        curr_pose = curr_kp + curr_kp_parent;
//        initial_pose = curr_kp_init + curr_kp_parent_init;
//    }
//    else
//    {
//        curr_pose = curr_kp;
//        initial_pose = curr_kp_init;
//    }

    double dev = norm(curr_kp-curr_kp_init);
//    double dev = norm(curr_pose-initial_pose);
//    Vector dev = curr_pose-initial_pose;
//    double deviation = dev[0]+dev[1]+dev[2];
    //    if(deviation > keypoints2conf[keypoint.getTag()].second)
//    {
//        yInfo() << keypoint.getTag().c_str()
//                << "(" << curr_kp[0] << "," << curr_kp[1] << "," << curr_kp[2] << ")"
//                << "(" << curr_kp_init[0] << "," << curr_kp_init[1] << "," << curr_kp_init[2] << ")"
//    //            << "(" << curr_kp_parent[0] << "," << curr_kp_parent[1] << "," << curr_kp_parent[2] << ")"
//    //            << "(" << curr_kp_child[0] << "," << curr_kp_child[1] << "," << curr_kp_child[2] << ")"
////                << "(" << curr_pose[0] << "," << curr_pose[1] << "," << curr_pose[2] << ")"
////                << "(" << initial_pose[0] << "," << initial_pose[1] << "," << initial_pose[2] << ")"
//                << dev;
//    }

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
                plane_normal_ = coronal;
    //            plane_normal = curr_skeleton.getCoronal()-kp_child;
    //            plane_normal /= norm(plane_normal);
                component_to_check = 0;
            }
            else if(rom->getTagPlane() == "sagittal")
            {
                plane_normal_ = sagittal;
    //            plane_normal = curr_skeleton.getSagittal()-kp_child;
    //            plane_normal /= norm(plane_normal);
                component_to_check = 1;
            }
            else if(rom->getTagPlane() == "transverse")
            {
                plane_normal_ = transverse;
    //            plane_normal = curr_skeleton.getSagittal()-kp_child;
    //            plane_normal /= norm(plane_normal);
                component_to_check = 2;
            }
            plane_normal_ = plane_normal_-kp_child;
            plane_normal_ /= norm(plane_normal_);

            ref_dir = rom->getRefDir();

            v1 = kp_child-kp_ref;
            score_exercise = 0.7;

            Vector k1(4,0.0);
            k1[0] = kp_ref[0];
            k1[1] = kp_ref[1];
            k1[2] = kp_ref[2];
            k1[3] = 1.0;
            Vector k2(4,0.0);
            k2[0] = kp_child[0];
            k2[1] = kp_child[1];
            k2[2] = kp_child[2];
            k2[3] = 1.0;
            Vector transformed_kp_ref = inv_reference_system*k1;
            Vector transformed_kp_child = inv_reference_system*k2;

            Vector diff = transformed_kp_child-transformed_kp_ref;

//            yInfo() << diff[component_to_check] << diff[1] << diff[2];
            if(abs(diff[component_to_check])>rom->getRangePlane())
            {
//                yInfo() << "out of the plane band" << diff[component_to_check];
                score_exercise = 0.4;
            }

            double dist = dot(v1,plane_normal_);
            v1 = v1-dist*plane_normal_;
            v1 /= norm(v1);

            double v1_norm = norm(v1);
            double v2_norm = norm(ref_dir);
            double dot_p = dot(v1,ref_dir);

            theta = acos(dot_p/(v1_norm*v2_norm));

//            plane_normal_=plane_normal;

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
