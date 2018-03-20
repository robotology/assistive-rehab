/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Manager.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include "Manager.h"
#include "Metric.h"
#include "Processor.h"

#include "src/motionAnalyzer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

void Manager::init()
{
    //tag2key initial configuration
    elbowLeft_init.resize(3);
    elbowRight_init.resize(3);
    handLeft_init.resize(3);
    handRight_init.resize(3);
    head_init.resize(3);
    shoulderCenter_init.resize(3);
    shoulderLeft_init.resize(3);
    shoulderRight_init.resize(3);
    hipLeft_init.resize(3);
    hipRight_init.resize(3);
    kneeLeft_init.resize(3);
    kneeRight_init.resize(3);
    ankleLeft_init.resize(3);
    ankleRight_init.resize(3);

    initial_keypoints.resize(14);

    elbowLeft.resize(3);
    elbowRight.resize(3);
    handLeft.resize(3);
    handRight.resize(3);
    head.resize(3);
    shoulderCenter.resize(3);
    shoulderLeft.resize(3);
    shoulderRight.resize(3);
    hipLeft.resize(3);
    hipRight.resize(3);
    kneeLeft.resize(3);
    kneeRight.resize(3);
    ankleLeft.resize(3);
    ankleRight.resize(3);

    curr_keypoints.resize(14);

    keypoints2conf[KeyPointTag::shoulder_center]="static";
    keypoints2conf[KeyPointTag::head]="static";
    keypoints2conf[KeyPointTag::shoulder_left]="static";
    keypoints2conf[KeyPointTag::elbow_left]="static";
    keypoints2conf[KeyPointTag::hand_left]="static";
    keypoints2conf[KeyPointTag::shoulder_right]="static";
    keypoints2conf[KeyPointTag::elbow_right]="static";
    keypoints2conf[KeyPointTag::hand_right]="static";
    keypoints2conf[KeyPointTag::hip_left]="static";
    keypoints2conf[KeyPointTag::knee_left]="static";
    keypoints2conf[KeyPointTag::ankle_left]="static";
    keypoints2conf[KeyPointTag::hip_right]="static";
    keypoints2conf[KeyPointTag::knee_right]="static";
    keypoints2conf[KeyPointTag::ankle_right]="static";

}

bool Manager::loadInitialConf(const string& motion_repertoire_file)
{
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext(this->rf->getContext().c_str());
    rf.setDefaultConfigFile(motion_repertoire_file.c_str()); //(this->rf->find("configuration-file").asString().c_str());
    rf.configure(0, NULL);

    Bottle &bGeneral = rf.findGroup("GENERAL");

    if(!bGeneral.isNull())
    {
//        nmovements = bGeneral.find("number_movements").asInt();
        if(Bottle *bElbowLeft_init = bGeneral.find("elbow_left_init_pose").asList())
        {
            elbowLeft_init[0] = bElbowLeft_init->get(0).asDouble();
            elbowLeft_init[1] = bElbowLeft_init->get(1).asDouble();
            elbowLeft_init[2] = bElbowLeft_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::elbow_left,elbowLeft_init));
        }
        else
            yError() << "Could not load initial pose for elbow left";

        if(Bottle *bElbowRight_init = bGeneral.find("elbow_right_init_pose").asList())
        {
            elbowRight_init[0] = bElbowRight_init->get(0).asDouble();
            elbowRight_init[1] = bElbowRight_init->get(1).asDouble();
            elbowRight_init[2] = bElbowRight_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::elbow_right,elbowRight_init));
        }
        else
            yError() << "Could not load initial pose for elbow right";

        if(Bottle *bHandLeft_init = bGeneral.find("hand_left_init_pose").asList())
        {
            handLeft_init[0] = bHandLeft_init->get(0).asDouble();
            handLeft_init[1] = bHandLeft_init->get(1).asDouble();
            handLeft_init[2] = bHandLeft_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::hand_left,handLeft_init));
        }
        else
            yError() << "Could not load initial pose for hand left";

        if(Bottle *bHandRight_init = bGeneral.find("hand_right_init_pose").asList())
        {
            handRight_init[0] = bHandRight_init->get(0).asDouble();
            handRight_init[1] = bHandRight_init->get(1).asDouble();
            handRight_init[2] = bHandRight_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::hand_right,handRight_init));
        }
        else
            yError() << "Could not load initial pose for hand right";

        if(Bottle *bHead_init = bGeneral.find("head_init_pose").asList())
        {
            head_init[0] = bHead_init->get(0).asDouble();
            head_init[1] = bHead_init->get(1).asDouble();
            head_init[2] = bHead_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::head,head_init));
        }
        else
            yError() << "Could not load initial pose for hand left";

        if(Bottle *bShoulderCenter_init = bGeneral.find("shoulder_center_init_pose").asList())
        {
            shoulderCenter_init[0] = bShoulderCenter_init->get(0).asDouble();
            shoulderCenter_init[1] = bShoulderCenter_init->get(1).asDouble();
            shoulderCenter_init[2] = bShoulderCenter_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::shoulder_center,shoulderCenter_init));
        }
        else
            yError() << "Could not load initial pose for shoulder center";

        if(Bottle *bShoulderLeft_init = bGeneral.find("shoulder_left_init_pose").asList())
        {
            shoulderLeft_init[0] = bShoulderLeft_init->get(0).asDouble();
            shoulderLeft_init[1] = bShoulderLeft_init->get(1).asDouble();
            shoulderLeft_init[2] = bShoulderLeft_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::shoulder_left,shoulderLeft_init));
        }
        else
            yError() << "Could not load initial pose for shoulder left";

        if(Bottle *bShoulderRight_init = bGeneral.find("shoulder_right_init_pose").asList())
        {
            shoulderRight_init[0] = bShoulderRight_init->get(0).asDouble();
            shoulderRight_init[1] = bShoulderRight_init->get(1).asDouble();
            shoulderRight_init[2] = bShoulderRight_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::shoulder_right,shoulderRight_init));
        }
        else
            yError() << "Could not load initial pose for shoulder right";

        if(Bottle *bHipLeft_init = bGeneral.find("hip_left_init_pose").asList())
        {
            hipLeft_init[0] = bHipLeft_init->get(0).asDouble();
            hipLeft_init[1] = bHipLeft_init->get(1).asDouble();
            hipLeft_init[2] = bHipLeft_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::hip_left,hipLeft_init));
        }
        else
            yError() << "Could not load initial pose for hip left";

        if(Bottle *bhipRight_init = bGeneral.find("hip_right_init_pose").asList())
        {
            hipRight_init[0] = bhipRight_init->get(0).asDouble();
            hipRight_init[1] = bhipRight_init->get(1).asDouble();
            hipRight_init[2] = bhipRight_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::hip_right,hipRight_init));
        }
        else
            yError() << "Could not load initial pose for hip right";

        if(Bottle *bKneeLeft_init = bGeneral.find("knee_left_init_pose").asList())
        {
            kneeLeft_init[0] = bKneeLeft_init->get(0).asDouble();
            kneeLeft_init[1] = bKneeLeft_init->get(1).asDouble();
            kneeLeft_init[2] = bKneeLeft_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::knee_left,kneeLeft_init));
        }
        else
            yError() << "Could not load initial pose for knee left";

        if(Bottle *bKneeRight_init = bGeneral.find("knee_right_init_pose").asList())
        {
            kneeRight_init[0] = bKneeRight_init->get(0).asDouble();
            kneeRight_init[1] = bKneeRight_init->get(1).asDouble();
            kneeRight_init[2] = bKneeRight_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::knee_right,kneeRight_init));
        }
        else
            yError() << "Could not load initial pose for knee right";

        if(Bottle *bAnkleLeft_init = bGeneral.find("ankle_left_init_pose").asList())
        {
            ankleLeft_init[0] = bAnkleLeft_init->get(0).asDouble();
            ankleLeft_init[1] = bAnkleLeft_init->get(1).asDouble();
            ankleLeft_init[2] = bAnkleLeft_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::ankle_left,ankleLeft_init));
        }
        else
            yError() << "Could not load initial pose for ankle left";

        if(Bottle *bAnkleRight_init = bGeneral.find("ankle_right_init_pose").asList())
        {
            ankleRight_init[0] = bAnkleRight_init->get(0).asDouble();
            ankleRight_init[1] = bAnkleRight_init->get(1).asDouble();
            ankleRight_init[2] = bAnkleRight_init->get(2).asDouble();
            initial_keypoints.push_back(make_pair(KeyPointTag::ankle_right,ankleRight_init));
        }
        else
            yError() << "Could not load initial pose for ankle right";

        initial_skeleton.update_fromstd(initial_keypoints);
    }
}

bool Manager::loadInitialConf(const Bottle& b)
{
    if(Bottle *bElbowLeft_init = b.find("elbow_left_init_pose").asList())
    {
        elbowLeft_init[0] = bElbowLeft_init->get(0).asDouble();
        elbowLeft_init[1] = bElbowLeft_init->get(1).asDouble();
        elbowLeft_init[2] = bElbowLeft_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::elbow_left)
                initial_keypoints[i] = make_pair(KeyPointTag::elbow_left,elbowLeft_init);
        }
        yInfo() << "Updated initial pose for elbow left";
    }

    if(Bottle *bElbowRight_init = b.find("elbow_right_init_pose").asList())
    {
        elbowRight_init[0] = bElbowRight_init->get(0).asDouble();
        elbowRight_init[1] = bElbowRight_init->get(1).asDouble();
        elbowRight_init[2] = bElbowRight_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::elbow_right)
                initial_keypoints[i] = make_pair(KeyPointTag::elbow_right,elbowRight_init);
        }
        yInfo() << "Updated initial pose for elbow right";
    }

    if(Bottle *bHandLeft_init = b.find("hand_left_init_pose").asList())
    {
        handLeft_init[0] = bHandLeft_init->get(0).asDouble();
        handLeft_init[1] = bHandLeft_init->get(1).asDouble();
        handLeft_init[2] = bHandLeft_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::hand_left)
                initial_keypoints[i] = make_pair(KeyPointTag::hand_left,handLeft_init);
        }
        yInfo() << "Updated initial pose for hand left";
    }

    if(Bottle *bHandRight_init = b.find("hand_right_init_pose").asList())
    {
        handRight_init[0] = bHandRight_init->get(0).asDouble();
        handRight_init[1] = bHandRight_init->get(1).asDouble();
        handRight_init[2] = bHandRight_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::hand_right)
                initial_keypoints[i] = make_pair(KeyPointTag::hand_right,handRight_init);
        }
        yInfo() << "Updated initial pose for hand right";
    }

    if(Bottle *bHead_init = b.find("head_init_pose").asList())
    {
        head_init[0] = bHead_init->get(0).asDouble();
        head_init[1] = bHead_init->get(1).asDouble();
        head_init[2] = bHead_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::head)
                initial_keypoints[i] = make_pair(KeyPointTag::head,head_init);
        }
        yInfo() << "Updated initial pose for head";
    }

    if(Bottle *bShoulderCenter_init = b.find("shoulder_center_init_pose").asList())
    {
        shoulderCenter_init[0] = bShoulderCenter_init->get(0).asDouble();
        shoulderCenter_init[1] = bShoulderCenter_init->get(1).asDouble();
        shoulderCenter_init[2] = bShoulderCenter_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::shoulder_center)
                initial_keypoints[i] = make_pair(KeyPointTag::shoulder_center,shoulderCenter_init);
        }
        yInfo() << "Updated initial pose for shoulder center";
    }

    if(Bottle *bShoulderLeft_init = b.find("shoulder_left_init_pose").asList())
    {
        shoulderLeft_init[0] = bShoulderLeft_init->get(0).asDouble();
        shoulderLeft_init[1] = bShoulderLeft_init->get(1).asDouble();
        shoulderLeft_init[2] = bShoulderLeft_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::shoulder_left)
                initial_keypoints[i] = make_pair(KeyPointTag::shoulder_left,shoulderLeft_init);
        }
        yInfo() << "Updated initial pose for shoulder left";
    }

    if(Bottle *bShoulderRight_init = b.find("shoulder_right_init_pose").asList())
    {
        shoulderRight_init[0] = bShoulderRight_init->get(0).asDouble();
        shoulderRight_init[1] = bShoulderRight_init->get(1).asDouble();
        shoulderRight_init[2] = bShoulderRight_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::shoulder_right)
                initial_keypoints[i] = make_pair(KeyPointTag::shoulder_right,shoulderRight_init);
        }
        yInfo() << "Updated initial pose for shoulder right";
    }

    if(Bottle *bHipLeft_init = b.find("hip_left_init_pose").asList())
    {
        hipLeft_init[0] = bHipLeft_init->get(0).asDouble();
        hipLeft_init[1] = bHipLeft_init->get(1).asDouble();
        hipLeft_init[2] = bHipLeft_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::hip_left)
                initial_keypoints[i] = make_pair(KeyPointTag::hip_left,hipLeft_init);
        }
        yInfo() << "Updated initial pose for hip left";
    }

    if(Bottle *bhipRight_init = b.find("hip_right_init_pose").asList())
    {
        hipRight_init[0] = bhipRight_init->get(0).asDouble();
        hipRight_init[1] = bhipRight_init->get(1).asDouble();
        hipRight_init[2] = bhipRight_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::hip_right)
                initial_keypoints[i] = make_pair(KeyPointTag::hip_right,hipRight_init);
        }
        yInfo() << "Updated initial pose for hip right";
    }

    if(Bottle *bKneeLeft_init = b.find("knee_left_init_pose").asList())
    {
        kneeLeft_init[0] = bKneeLeft_init->get(0).asDouble();
        kneeLeft_init[1] = bKneeLeft_init->get(1).asDouble();
        kneeLeft_init[2] = bKneeLeft_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::knee_left)
                initial_keypoints[i] = make_pair(KeyPointTag::knee_left,kneeLeft_init);
        }
        yInfo() << "Updated initial pose for knee left";
    }

    if(Bottle *bKneeRight_init = b.find("knee_right_init_pose").asList())
    {
        kneeRight_init[0] = bKneeRight_init->get(0).asDouble();
        kneeRight_init[1] = bKneeRight_init->get(1).asDouble();
        kneeRight_init[2] = bKneeRight_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::knee_right)
                initial_keypoints[i] = make_pair(KeyPointTag::knee_right,kneeRight_init);
        }
        yInfo() << "Updated initial pose for knee right";
    }

    if(Bottle *bAnkleLeft_init = b.find("ankle_left_init_pose").asList())
    {
        ankleLeft_init[0] = bAnkleLeft_init->get(0).asDouble();
        ankleLeft_init[1] = bAnkleLeft_init->get(1).asDouble();
        ankleLeft_init[2] = bAnkleLeft_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::ankle_left)
                initial_keypoints[i] = make_pair(KeyPointTag::ankle_left,ankleLeft_init);
        }
        yInfo() << "Updated initial pose for ankle left";
    }

    if(Bottle *bAnkleRight_init = b.find("ankle_right_init_pose").asList())
    {
        ankleRight_init[0] = bAnkleRight_init->get(0).asDouble();
        ankleRight_init[1] = bAnkleRight_init->get(1).asDouble();
        ankleRight_init[2] = bAnkleRight_init->get(2).asDouble();

        for(int i=0; i<initial_keypoints.size(); i++)
        {
            if(initial_keypoints[i].first == KeyPointTag::ankle_right)
                initial_keypoints[i] = make_pair(KeyPointTag::ankle_right,ankleRight_init);
        }
        yInfo() << "Updated initial pose for ankle right";
    }

    initial_skeleton.update(initial_keypoints);
}

bool Manager::loadMotionList(const string& motion_repertoire_file)
{
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext(this->rf->getContext().c_str());
    rf.setDefaultConfigFile(motion_repertoire_file.c_str()); //(this->rf->find("configuration-file").asString().c_str());
    rf.configure(0, NULL);

    Bottle &bGeneral = rf.findGroup("GENERAL");

    if(!bGeneral.isNull())
    {
        if(Bottle *motion_tag = bGeneral.find("motion_tag").asList())
        {
            if(Bottle *n_motion_tag = bGeneral.find("number_motion").asList())
            {
                for(int i=0; i<motion_tag->size(); i++)
                {
                    string curr_tag = motion_tag->get(i).asString();
                    int motion_number = n_motion_tag->get(i).asInt();

                    for(int j=0; j<motion_number; j++)
                    {
                        Bottle &bMotion = rf.findGroup(curr_tag+"_"+to_string(j));
                        if(!bMotion.isNull())
                        {
                            Metric* newMetric;
                            if(curr_tag == "ROM") //Rom_Processor::motion_type)
                            {
                                string motion_type = bMotion.find("motion_type").asString();
                                string tag_joint = bMotion.find("tag_joint").asString();
                                double min = bMotion.find("min").asDouble();
                                double max = bMotion.find("max").asDouble();
                                double timeout = bMotion.find("timeout").asDouble();

                                Vector ref_dir;
                                ref_dir.resize(3);
                                if(Bottle *bRefdir = bMotion.find("ref_dir").asList())
                                {
                                    ref_dir[0] = bRefdir->get(0).asDouble();
                                    ref_dir[1] = bRefdir->get(1).asDouble();
                                    ref_dir[2] = bRefdir->get(2).asDouble();
                                }
                                else
                                    yError() << "Could not find reference direction";

                                Vector plane_normal;
                                plane_normal.resize(3);
                                if(Bottle *bPlane = bMotion.find("plane_normal").asList())
                                {
                                    plane_normal[0] = bPlane->get(0).asDouble();
                                    plane_normal[1] = bPlane->get(1).asDouble();
                                    plane_normal[2] = bPlane->get(2).asDouble();
                                }
                                else
                                    yError() << "Could not find reference plane";

                                newMetric = new Rom(motion_type, tag_joint, ref_dir, plane_normal, min, max, timeout);
//                                metrics.push_back(newMetric);

                                //overwrites initial configuration if different
                                loadInitialConf(bMotion);

                                Bottle *elbowLC = bMotion.find("elbow_left_configuration").asList();
                                if(elbowLC)
                                {
                                    string eLC = elbowLC->get(0).asString();
                                    if(eLC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::elbow_left] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load elbow left configuration";

                                Bottle *elbowRC = bMotion.find("elbow_right_configuration").asList();
                                if(elbowRC)
                                {
                                    string eRC = elbowRC->get(0).asString();
                                    if(eRC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::elbow_right] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load elbow right configuration";

                                Bottle *handLC = bMotion.find("hand_left_configuration").asList();
                                if(handLC)
                                {
                                    string hnLC = handLC->get(0).asString();
                                    if(hnLC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::hand_left] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load hand left configuration";

                                Bottle *handRC = bMotion.find("hand_right_configuration").asList();
                                if(handRC)
                                {
                                    string hnRC = handRC->get(0).asString();
                                    if(hnRC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::hand_right] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load hand right configuration";

                                Bottle *headC = bMotion.find("head_configuration").asList();
                                if(headC)
                                {
                                    string hC = headC->get(0).asString();
                                    if(hC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::head] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load head configuration";

                                Bottle *shoulderCC = bMotion.find("shoulder_center_configuration").asList();
                                if(shoulderCC)
                                {
                                    string sCC = shoulderCC->get(0).asString();
                                    if(sCC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::shoulder_center] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load shoulder center configuration";

                                Bottle *shoulderLC = bMotion.find("shoulder_left_configuration").asList();
                                if(shoulderLC)
                                {
                                    string sLC = shoulderLC->get(0).asString();
                                    if(sLC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::shoulder_left] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load shoulder left configuration";

                                Bottle *shoulderRC = bMotion.find("shoulder_right_configuration").asList();
                                if(shoulderRC)
                                {
                                    string sRC = shoulderRC->get(0).asString();
                                    if(sRC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::shoulder_right] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load shoulder right configuration";

                                Bottle *hipLC = bMotion.find("hip_left_configuration").asList();
                                if(hipLC)
                                {
                                    string hLC = hipLC->get(0).asString();
                                    if(hLC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::hip_left] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load hip left configuration";

                                Bottle *hipRC = bMotion.find("hip_right_configuration").asList();
                                if(hipRC)
                                {
                                    string hRC = hipRC->get(0).asString();
                                    if(hRC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::hip_right] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load hip right configuration";

                                Bottle *kneeLC = bMotion.find("knee_left_configuration").asList();
                                if(kneeLC)
                                {
                                    string kLC = kneeLC->get(0).asString();
                                    if(kLC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::knee_left] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load knee left configuration";

                                Bottle *kneeRC = bMotion.find("knee_right_configuration").asList();
                                if(kneeRC)
                                {
                                    string kRC = kneeRC->get(0).asString();
                                    if(kRC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::knee_right] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load knee right configuration";

                                Bottle *ankleLC = bMotion.find("ankle_left_configuration").asList();
                                if(ankleLC)
                                {
                                    string aLC = ankleLC->get(0).asString();
                                    if(aLC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::ankle_left] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load ankle left configuration";

                                Bottle *ankleRC = bMotion.find("ankle_right_configuration").asList();
                                if(ankleRC)
                                {
                                    string aRC = ankleRC->get(0).asString();
                                    if(aRC == "mobile")
                                    {
                                        keypoints2conf[KeyPointTag::ankle_right] = "mobile";
                                    }
                                }
                                else
                                    yError() << "Could not load ankle right configuration";
                            }

                            //add the current metric to the repertoire
                            motion_repertoire.insert(pair<string, Metric*>(curr_tag+"_"+to_string(j), newMetric)); // metrics[j]));
//                            motion_repertoire[curr_tag+"_"+to_string(j)]->print();
                        }
                    }
                }
            }
        }
    }
    else
    {
        yError() << "Error in loading parameters. Stopping module!";
        return false;
    }

    return true;
}

/********************************************************/
bool Manager::loadSequence(const string &sequencer_file)
{
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext(this->rf->getContext().c_str());
    rf.setDefaultConfigFile(sequencer_file.c_str()); //(this->rf->find("configuration-file").asString().c_str());
    rf.configure(0, NULL);

    if(Bottle *metric_type = rf.find("metric_type").asList())
    {
        if(Bottle *motion_type = rf.find("motion_type").asList())
        {
            metrics.resize(motion_type->size());
            processors.resize(motion_type->size());

            if(Bottle *tag_joint = rf.find("tag_joint").asList())
            {
                if(Bottle *n_rep = rf.find("number_repetitions").asList())
                {
                    for(int i=0; i<motion_type->size(); i++)
                    {
                        string single_motion = motion_type->get(i).asString();
                        string joint = tag_joint->get(i).asString();
                        string metric_tag = metric_type->get(i).asString();
                        int n = n_rep->get(i).asInt();

                        yInfo() << "Exercise to perform:" << n << single_motion << "for" << joint;

                        metrics[i] = motion_repertoire.at(metric_tag);
                        metrics[i]->print();

                        Processor* newProcessor = createProcessor(metric_tag, metrics[i]);
                        newProcessor->setInitialConf(initial_skeleton, keypoints2conf);
                        processors[i] = newProcessor;
                    }
                }
            }
        }
    }
}

/********************************************************/
void Manager::getKeyframes()
{
    curr_keypoints.clear();

    //ask for the property id
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("ask"));
    Bottle &content = cmd.addList().addList();
    content.addString("body");

//    yInfo() << "Query opc: " << cmd.toString();
    opcPort.write(cmd, reply);
//    yInfo() << "Reply from opc:" << reply.toString();

    if(reply.size() > 1)
    {
        if(reply.get(0).asVocab() == Vocab::encode("ack"))
        {
            if(Bottle *idField = reply.get(1).asList())
            {
                if(Bottle *idValues = idField->get(1).asList())
                {
                    //get the last id
                    int id = idValues->get(idValues->size()-1).asInt();
                    //                        yInfo() << id;

                    //given the id, get the value of the property
                    cmd.clear();
                    cmd.addVocab(Vocab::encode("get"));
                    Bottle &content = cmd.addList().addList();
                    content.addString("id");
                    content.addInt(id);
                    Bottle replyProp;

                    //                        yInfo() << "Command sent to the port: " << cmd.toString();
                    opcPort.write(cmd, replyProp);
                    //                        yInfo() << "Reply from opc:" << replyProp.toString();

                    if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                    {
                        if(Bottle *propField = replyProp.get(1).asList())
                        {
                            if(Bottle *propSubField = propField->find("body").asList())
                            {
                                if(Bottle *elbowLeftB = propSubField->find(KeyPointTag::elbow_left).asList())
                                {
                                    elbowLeft[0] = elbowLeftB->get(0).asDouble();
                                    elbowLeft[1] = elbowLeftB->get(1).asDouble();
                                    elbowLeft[2] = elbowLeftB->get(2).asDouble();
                                    //                                    yInfo() << elbowLeft[0] << elbowLeft[1] << elbowLeft[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::elbow_left,elbowLeft));
                                }
                                else
                                    yWarning() << "Could not read elbow left from OPC.. not visible?";

                                if(Bottle *elbowRightB = propSubField->find(KeyPointTag::elbow_right).asList())
                                {
                                    elbowRight[0] = elbowRightB->get(0).asDouble();
                                    elbowRight[1] = elbowRightB->get(1).asDouble();
                                    elbowRight[2] = elbowRightB->get(2).asDouble();
                                    //                                    yInfo() << elbowRight[0] << elbowRight[1] << elbowRight[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::elbow_right,elbowRight));
                                }
                                else
                                    yWarning() << "Could not read elbow right from OPC.. not visible?";

                                if(Bottle *handLeftB = propSubField->find(KeyPointTag::hand_left).asList())
                                {
                                    handLeft[0] = handLeftB->get(0).asDouble();
                                    handLeft[1] = handLeftB->get(1).asDouble();
                                    handLeft[2] = handLeftB->get(2).asDouble();
                                    //                                    yInfo() << handLeft[0] << handLeft[1] << handLeft[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::hand_left,handLeft));
                                }
                                else
                                    yWarning() << "Could not read hand left from OPC.. not visible?";

                                if(Bottle *handRightB = propSubField->find(KeyPointTag::hand_right).asList())
                                {
                                    handRight[0] = handRightB->get(0).asDouble();
                                    handRight[1] = handRightB->get(1).asDouble();
                                    handRight[2] = handRightB->get(2).asDouble();
                                    //                                    yInfo() << handRight[0] << handRight[1] << handRight[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::hand_right,handRight));
                                }
                                else
                                    yWarning() << "Could not read hand right from OPC.. not visible?";

                                if(Bottle *headB = propSubField->find(KeyPointTag::head).asList())
                                {
                                    head[0] = headB->get(0).asDouble();
                                    head[1] = headB->get(1).asDouble();
                                    head[2] = headB->get(2).asDouble();
                                    //                                    yInfo() << head[0] << head[1] << head[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::head,head));
                                }
                                else
                                    yWarning() << "Could not read hand left from OPC.. not visible?";

                                if(Bottle *shoulderCenterB = propSubField->find(KeyPointTag::shoulder_center).asList())
                                {
                                    shoulderCenter[0] = shoulderCenterB->get(0).asDouble();
                                    shoulderCenter[1] = shoulderCenterB->get(1).asDouble();
                                    shoulderCenter[2] = shoulderCenterB->get(2).asDouble();
                                    //                                    yInfo() << shoulderCenter[0] << shoulderCenter[1] << shoulderCenter[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::shoulder_center,shoulderCenter));
                                }
                                else
                                    yWarning() << "Could not read shoulder center from OPC.. not visible?";

                                if(Bottle *shoulderLeftB = propSubField->find(KeyPointTag::shoulder_left).asList())
                                {
                                    shoulderLeft[0] = shoulderLeftB->get(0).asDouble();
                                    shoulderLeft[1] = shoulderLeftB->get(1).asDouble();
                                    shoulderLeft[2] = shoulderLeftB->get(2).asDouble();
                                    //                                    yInfo() << shoulderLeft[0] << shoulderLeft[1] << shoulderLeft[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::shoulder_left,shoulderLeft));
                                }
                                else
                                    yWarning() << "Could not read shoulder left from OPC.. not visible?";

                                if(Bottle *shoulderRightB = propSubField->find(KeyPointTag::shoulder_right).asList())
                                {
                                    shoulderRight[0] = shoulderRightB->get(0).asDouble();
                                    shoulderRight[1] = shoulderRightB->get(1).asDouble();
                                    shoulderRight[2] = shoulderRightB->get(2).asDouble();
                                    //                                    yInfo() << shoulderRight[0] << shoulderRight[1] << shoulderRight[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::shoulder_right,shoulderRight));
                                }
                                else
                                    yWarning() << "Could not read shoulder right from OPC.. not visible?";

                                if(Bottle *hipLeftB = propSubField->find(KeyPointTag::hip_left).asList())
                                {
                                    hipLeft[0] = hipLeftB->get(0).asDouble();
                                    hipLeft[1] = hipLeftB->get(1).asDouble();
                                    hipLeft[2] = hipLeftB->get(2).asDouble();
                                    //                                    yInfo() << hipLeft[0] << hipLeft[1] << hipLeft[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::hip_left,hipLeft));
                                }
                                else
                                    yWarning() << "Could not read hip left from OPC.. not visible?";

                                if(Bottle *hipRightB = propSubField->find(KeyPointTag::hip_right).asList())
                                {
                                    hipRight[0] = hipRightB->get(0).asDouble();
                                    hipRight[1] = hipRightB->get(1).asDouble();
                                    hipRight[2] = hipRightB->get(2).asDouble();
                                    //                                    yInfo() << hipRight[0] << hipRight[1] << hipRight[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::hip_right,hipRight));
                                }
                                else
                                    yWarning() << "Could not read hip right from OPC.. not visible?";

                                if(Bottle *kneeLeftB = propSubField->find(KeyPointTag::knee_left).asList())
                                {
                                    kneeLeft[0] = kneeLeftB->get(0).asDouble();
                                    kneeLeft[1] = kneeLeftB->get(1).asDouble();
                                    kneeLeft[2] = kneeLeftB->get(2).asDouble();
                                    //                                    yInfo() << kneeLeft[0] << kneeLeft[1] << kneeLeft[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::knee_left,kneeLeft));
                                }
                                else
                                    yWarning() << "Could not read knee left from OPC.. not visible?";

                                if(Bottle *kneeRightB = propSubField->find(KeyPointTag::knee_right).asList())
                                {
                                    kneeRight[0] = kneeRightB->get(0).asDouble();
                                    kneeRight[1] = kneeRightB->get(1).asDouble();
                                    kneeRight[2] = kneeRightB->get(2).asDouble();
                                    //                                    yInfo() << kneeRight[0] << kneeRight[1] << kneeRight[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::knee_right,kneeRight));
                                }
                                else
                                    yWarning() << "Could not read knee right from OPC.. not visible?";

                                if(Bottle *ankleLeftB = propSubField->find(KeyPointTag::ankle_left).asList())
                                {
                                    ankleLeft[0] = ankleLeftB->get(0).asDouble();
                                    ankleLeft[1] = ankleLeftB->get(1).asDouble();
                                    ankleLeft[2] = ankleLeftB->get(2).asDouble();
                                    //                                    yInfo() << kneeLeft[0] << kneeLeft[1] << kneeLeft[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::ankle_left,ankleLeft));
                                }
                                else
                                    yWarning() << "Could not read ankle left from OPC.. not visible?";

                                if(Bottle *ankleRightB = propSubField->find(KeyPointTag::ankle_right).asList())
                                {
                                    ankleRight[0] = ankleRightB->get(0).asDouble();
                                    ankleRight[1] = ankleRightB->get(1).asDouble();
                                    ankleRight[2] = ankleRightB->get(2).asDouble();
                                    //                                    yInfo() << kneeRight[0] << kneeRight[1] << kneeRight[2];

                                    curr_keypoints.push_back(make_pair(KeyPointTag::ankle_right,ankleRight));
                                }
                                else
                                    yWarning() << "Could not read ankle right from OPC.. not visible?";

                            }
                        }
                    }
                }
            }
        }
    }
}

/********************************************************/
bool Manager::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/********************************************************/
bool Manager::configure(ResourceFinder &rf)
{
    this->rf = &rf;
    string moduleName = rf.check("name", Value("motionAnalyzer")).asString();
    setName(moduleName.c_str());

    string robot = rf.check("robot", Value("icub")).asString();

    string motion_repertoire_file = rf.check("repertoire_file", Value("motion-repertoire.ini")).asString();
    string sequencer_file = rf.check("sequencer_file", Value("sequencer.ini")).asString();

    opcPort.open(("/" + getName() + "/opc").c_str());
    scopePort.open(("/" + getName() + "/scope").c_str());
    rpcPort.open(("/" + getName() + "/cmd").c_str());
    attach(rpcPort);

    init();
    loadInitialConf(motion_repertoire_file);
    if(!loadMotionList(motion_repertoire_file))
        return false;
    loadSequence(sequencer_file);

    tstart = Time::now();

    return true;
}

/********************************************************/
bool Manager::close()
{
    for(int j=0; j<metrics.size(); j++)
        delete metrics[j];

    for(int i=0; i<processors.size(); i++)
        delete processors[i];

    opcPort.close();
    scopePort.close();
    rpcPort.close();

    return true;
}

/********************************************************/
double Manager::getPeriod()
{
    return 1.0;
}

/********************************************************/
bool Manager::updateModule()
{
    //if we query the database
    if(opcPort.getOutputCount() > 0)
    {
        //get keyframes from skeleton
        getKeyframes();

        //populate Skeleton data structure
        SkeletonWaist skeleton;
        skeleton.update_fromstd(curr_keypoints);

//        skeleton.print();

        //transform detected skeleton into standard
        skeleton.normalize();

        Vector result;
        result.resize(processors.size());
        double tnow = Time::now();
        double currres = -1.0;
        double prev_timeout = 0.0;
        double timeout = 0.0;
        for(int i=0; i<processors.size(); i++)
        {
            processors[i]->update(skeleton);
            if(processors[i]->isDeviatingFromIntialPose())
                yWarning() << "Deviating from initial pose\n";

            result[i] = processors[i]->computeMetric();

            //check which is the current processor based on timeout
            timeout += processors[i]->getTimeout();
            //            cout << (tnow-tstart) << " " << prev_timeout << " " << timeout << endl;
            if( prev_timeout < (tnow-tstart) && (tnow-tstart) < timeout )
                currres = result[i];

            prev_timeout = timeout;
        }

        //write on output port
        Bottle &scopebottleout = scopePort.prepare();
        scopebottleout.clear();
        scopebottleout.addDouble(currres);
        scopePort.write();

    }

    return true;
}

