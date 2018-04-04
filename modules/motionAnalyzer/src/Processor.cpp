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

#define CONST (4/3)*M_PI

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

}

//void Processor::setInitialConf(const SkeletonStd &skeleton_init_, const map<string, string> &keypoints2conf_)
//{
//    skeleton_init = skeleton_init_;
//    keypoints2conf = keypoints2conf_;
//}

bool Processor::isStatic(const KeyPoint& keypoint)
{
    if(keypoints2conf[keypoint.getTag()].first == "static")
        return true;
    else
        return false;
}

void Processor::update(const SkeletonWaist &curr_skeleton_)
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

//                    cout << keypoint.getTag() << " " << keypoint_child->getTag() << " " << keypoint_parent->getTag() << " "
//                         << "(" << curr_kp[0] << " " << curr_kp[1] << " " << curr_kp[2] << ") "
//                         << "(" << curr_kp_child[0] << " " << curr_kp_child[1] << " " << curr_kp_child[2] << ") "
//                         << "(" << curr_kp_parent[0] << " " << curr_kp_parent[1] << " " << curr_kp_parent[2] << ") "
//                         << curr_pose[0] << " " << curr_pose[1] << " " << curr_pose[2] << " "
//                         << initial_pose[0] << " " << initial_pose[1] << " " << initial_pose[2]
//                         << " " << deviation << endl;

                    if((deviation) > CONST*keypoints2conf[keypoint.getTag()].second)
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

//                cout << keypoint.getTag() << " " << deviation << endl;

                if((deviation) > CONST*keypoints2conf[keypoint.getTag()].second)
                {
                    isDeviating = true;
                }
            }
        }
    }
//    cout << endl;

    return isDeviating;
}

/********************************************************/
Rom_Processor::Rom_Processor()
{

}

Rom_Processor::Rom_Processor(const Metric *rom_)
{
    rom = (Rom*)rom_;
}

void Rom_Processor::setInitialConf(const SkeletonWaist &skeleton_init_, const map<string, pair<string, double> > &keypoints2conf_)
{
    skeleton_init = skeleton_init_;
    keypoints2conf = keypoints2conf_;
}

double Rom_Processor::computeMetric()
{
    //get reference keypoint from skeleton
    string tag_joint = rom->getTagJoint();
    const KeyPoint *keypoint_ref = curr_skeleton[tag_joint];
    Vector kp_ref = keypoint_ref->getPoint();

    Vector v1;
    if(keypoint_ref->getNumChild())
    {
        const KeyPoint *keypoint_child = keypoint_ref->getChild(0);
        Vector kp_child = keypoint_child->getPoint();

        Vector plane_normal = rom->getPlane();
        v1 = kp_child-kp_ref;
        double dist = dot(v1, plane_normal);
        v1 = v1-dist*plane_normal;

        Vector ref_dir = rom->getRefDir();

        double v1_norm = norm(v1);
        double v2_norm = norm(ref_dir);
        double dot_p = dot(v1, ref_dir);
        return ( acos(dot_p/(v1_norm*v2_norm)) * (180/M_PI) );
    }
    else
    {
        yError() << "The keypoint does not have a child ";
        return 0.0;
    }
}
