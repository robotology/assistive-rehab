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

#define MAX_DEVIATION_FROM_INITIAL_POSE 1

using namespace std;
using namespace yarp::math;
using namespace assistive_rehab;

const string Rom_Processor::motion_type = "ROM";

Processor* createProcessor(const string& motion_tag, const Metric* metric_)
{
    if(motion_tag == Rom_Processor::motion_type)
    {
        yInfo() << "Creating processor for" << Rom_Processor::motion_type << "\n";
        return new Rom_Processor(metric_);
    }
}

/********************************************************/
Processor::Processor()
{
    xy_normal.resize(3);
    yz_normal.resize(3);
    xy_normal[0] = 0.0; xy_normal[1] = 0.0; xy_normal[2] = 1.0;
    yz_normal[0] = 1.0; yz_normal[1] = 0.0; yz_normal[2] = 0.0;
}

void Processor::setInitialConf(const SkeletonStd &skeleton_init_, const map<string, string> &keypoints2conf_)
{
    skeleton_init = skeleton_init_;
    keypoints2conf = keypoints2conf_;
}

bool Processor::isStatic(const KeyPoint& keypoint)
{
    if(keypoints2conf[keypoint.getTag()] == "static")
        return true;
    else
        return false;
}

void Processor::update(const SkeletonStd &curr_skeleton_)
{
    curr_skeleton = curr_skeleton_;
}

bool Processor::isDeviatingFromIntialPose()
{
    bool isDeviating = false;
    for(unsigned int i=0; i<curr_skeleton.getNumKeyPoints(); i++)
    {
        //get current keypoint
        const KeyPoint* keypoint = curr_skeleton[i];
        const KeyPoint* keypoint_init = skeleton_init[i];

        if(isStatic(*keypoint) && isDeviatingFromIntialPose(*keypoint, *keypoint_init))
        {
            isDeviating = true;
//            yWarning() << "Deviating from initial pose";
        }
    }

    return isDeviating;
}

bool Processor::isDeviatingFromIntialPose(const KeyPoint& keypoint, const KeyPoint& keypoint_init)
{
    bool isDeviating = false;
    Vector curr_kp, curr_kp_parent, curr_kp_child,
            curr_kp_init, curr_kp_parent_init, curr_kp_child_init;
    curr_kp.resize(3);
    curr_kp_parent.resize(3);
    curr_kp_child.resize(3);
    curr_kp_init.resize(3);
    curr_kp_parent_init.resize(3);
    curr_kp_child_init.resize(3);

    curr_kp = keypoint.getPoint();
    curr_kp_init = keypoint_init.getPoint();

    int nParents = keypoint.getNumParent();
    int nChildren = keypoint.getNumChild();
    if(nParents)
    {
        for(int j=0; j<nParents; j++)
        {
            //standard skeleton
            const KeyPoint* keypoint_parent = keypoint.getParent(j);
            curr_kp_parent = keypoint_parent->getPoint();

            //initial standard skeleton
            const KeyPoint* keypoint_parent_init = keypoint_init.getParent(j);
            curr_kp_parent_init = keypoint_parent_init->getPoint();

            if(nChildren)
            {
                for(int k=0; k<nChildren; k++)
                {
                    const KeyPoint* keypoint_child = keypoint.getChild(k);
                    curr_kp_child = keypoint_child->getPoint();

                    const KeyPoint* keypoint_child_init = keypoint_init.getChild(k);
                    curr_kp_child_init = keypoint_child_init->getPoint();

                    //compute deviation from initial pose
                    Vector curr_pose = curr_kp + curr_kp_parent + curr_kp_child;
                    Vector initial_pose = curr_kp_init + curr_kp_parent_init + curr_kp_child_init;
                    double deviation = fabs(fabs(curr_pose[0]+curr_pose[1]+curr_pose[2])-fabs(initial_pose[0]+initial_pose[1]+initial_pose[2]));

                    if((deviation) > MAX_DEVIATION_FROM_INITIAL_POSE)
                    {
                        isDeviating = true;
                    }
                }
            }
            else
            {
                //compute deviation from initial pose
                Vector curr_pose = curr_kp + curr_kp_parent + curr_kp_child;
                Vector initial_pose = curr_kp_init + curr_kp_parent_init + curr_kp_child_init;
                double deviation = fabs(fabs(curr_pose[0]+curr_pose[1]+curr_pose[2])-fabs(initial_pose[0]+initial_pose[1]+initial_pose[2]));

                if((deviation) > MAX_DEVIATION_FROM_INITIAL_POSE)
                {
                    isDeviating = true;
                }
            }
        }
    }

    return isDeviating;
}

/********************************************************/
Rom_Processor::Rom_Processor()
{

}

Rom_Processor::Rom_Processor(const Metric *rom_)
{
    rom = (Rom*)rom_;
//    cout << "processing rom id " << rom->getIdJoint() << endl;
}

double Rom_Processor::computeMetric()
{
    //get reference keypoint from skeleton
    string tag_joint = rom->getTagJoint();
    const KeyPoint *keypoint_ref = curr_skeleton[tag_joint];
    Vector kp_ref = keypoint_ref->getPoint();
    Vector dir_ref(3, 0.0);
    dir_ref[0] = 0.0;
    dir_ref[1] = -1.0;
    dir_ref[2] = 0.0;

    Vector v1, v2;
    if(keypoint_ref->getNumChild())
    {
        const KeyPoint *keypoint_child = keypoint_ref->getChild(0);
        Vector kp_child = keypoint_child->getPoint();
        v1 = kp_child-kp_ref;

        if(rom->getMotionType() == "abduction") //abduction/adduction
        {
            //project vector onto xy plane
            double dist = dot(v1, xy_normal);
            v1 = v1-dist*xy_normal;
            v2 = dir_ref;
        }
        else if(rom->getMotionType() == "flexion") //flexion/extension
        {
            //project vector onto yz plane
            double dist = dot(v1, yz_normal);
            v1 = v1-dist*yz_normal;
            v2 = dir_ref;
        }
        else if(rom->getMotionType() == "rotation") //internal/external rotation
        {
            const KeyPoint *keypoint_parent = keypoint_ref->getParent(0);
            Vector kp_parent = keypoint_parent->getPoint();
            v2 = kp_ref-kp_parent;

            //project vector onto xy plane
            double dist = dot(v1, xy_normal);
            v1 = v1-dist*xy_normal;
        }

        double v1_norm = norm(v1);
        double v2_norm = norm(v2);
        double dot_p = dot(v1, v2);
        return ( acos(dot_p/(v1_norm*v2_norm)) * (180/M_PI) );
    }
}
