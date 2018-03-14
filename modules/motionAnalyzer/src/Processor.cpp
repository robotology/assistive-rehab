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

Processor::Processor(const SkeletonStd &skeleton_init_)
{
    skeleton_init = skeleton_init_;
}

bool Processor::isDeviatingFromIntialPose(SkeletonStd& curr_skeleton)
{
    bool isDeviating = false;
    for(unsigned int i=0; i<curr_skeleton.getNumKeyPoints(); i++)
    {
        //get current keypoint
        const KeyPoint* keypoint = curr_skeleton[i];
        const KeyPoint* keypoint_init = skeleton_init[i];

//        if(keypoint->isStationary() && isDeviatingFromIntialPose(*keypoint, *keypoint_init))
        if(isDeviatingFromIntialPose(*keypoint, *keypoint_init))
        {
            isDeviating = true;
            yWarning() << "keypoint" << keypoint->getTag() << "is deviating from initial pose";
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
                }
            }

            //compute deviation from initial pose
            Vector curr_pose = curr_kp + curr_kp_parent + curr_kp_child;
            Vector initial_pose = curr_kp_init + curr_kp_parent_init + curr_kp_child_init;
            Vector deviation = curr_pose - initial_pose;

            if((deviation[0]+deviation[1]+deviation[2]) > 1)
                isDeviating = true;
        }
    }

    return isDeviating;
}

/********************************************************/
Rom_Processor::Rom_Processor(Rom *rom_)
{
    rom = rom_;
}

double Rom_Processor::computeRom()
{
    //get keypoint from skeleton
    int id = rom->getIdJoint();
//    KeyPoint *keypoint = rom->getSkeleton()[id];
//    Vector kp = keypoint->getPoint();

//    Vector kp_parent, kp_child;

//    //get child and parent
//    if(id == 0 || id == 1) //if shoulder: change id with id shoulder
//    {
//        KeyPoint *keypoint_parent = keypoint->getParent(1);
//        kp_parent = keypoint_parent->getPoint();
//        KeyPoint *keypoint_child = keypoint->getChild(0);
//        kp_child = keypoint_child->getPoint();
//    }
//    else if(id == 2) //if elbow
//    {
//        KeyPoint *keypoint_parent = keypoint->getParent(0);
//        kp_parent = keypoint_parent->getPoint();
//        KeyPoint *keypoint_child = keypoint->getChild(0);
//        kp_child = keypoint_child->getPoint();
//    }

//    double a_norm = norm(kp-kp_parent);
//    double b_norm = norm(kp-kp_child);

//    double dot_p = dot(kp-kp_parent, kp-kp_child);

//    return ( acos(dot_p/(a_norm*b_norm)) * (180/M_PI) );

    return true;
}
