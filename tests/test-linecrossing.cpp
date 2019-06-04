/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-linecrossing.cpp
 * @authors: Valentina Vasco
 */

#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Vocab.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class CrossingChecker : public RFModule
{
public:

    SkeletonStd skeletonIn;
    BufferedPort<Bottle> scopePort;
    RpcClient opcPort;
    double threshold;

    /********************************************************/
    bool configure(ResourceFinder &rf) override
    {
        threshold=rf.check("threshold",Value(0.25)).asDouble();
        scopePort.open("/test-linecrossing/scope:o");
        opcPort.open("/test-linecrossing/cmd:opc");

        return true;
    }

    /********************************************************/
    double getPeriod() override
    {
        return 0.1;
    }

    /********************************************************/
    bool getSkeleton()
    {
        //ask for the property id
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList().addList();
        content.addString("skeleton");

        opcPort.write(cmd,reply);
        if(reply.size()>1)
        {
            if(reply.get(0).asVocab() == Vocab::encode("ack"))
            {
                if(Bottle *idField=reply.get(1).asList())
                {
                    if(Bottle *idValues=idField->get(1).asList())
                    {
                        for(int i=0; i<idValues->size(); i++)
                        {
                            int id=idValues->get(i).asInt();

                            //given the id, get the value of the property
                            cmd.clear();
                            cmd.addVocab(Vocab::encode("get"));
                            Bottle &content = cmd.addList().addList();
                            Bottle replyProp;
                            content.addString("id");
                            content.addInt(id);
                            opcPort.write(cmd,replyProp);
                            if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                            {
                                if(Bottle *propField = replyProp.get(1).asList())
                                {
                                    Property prop(propField->toString().c_str());
                                    string tag=prop.find("tag").asString();
                                    if(!tag.empty())
                                    {
                                        if(prop.check("tag"))
                                        {
                                            Skeleton* skeleton = skeleton_factory(prop);
                                            skeletonIn.update(skeleton->toProperty());
                                            delete skeleton;
                                            return true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

    /********************************************************/
    bool getLinePose(Vector &est_pose)
    {
        //ask for the property id
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList().addList();
        content.addString("finish-line");

        opcPort.write(cmd,reply);
        if(reply.size()>1)
        {
            if(reply.get(0).asVocab() == Vocab::encode("ack"))
            {
                if(Bottle *idField=reply.get(1).asList())
                {
                    if(Bottle *idVal=idField->get(1).asList())
                    {
                        int id=idVal->get(0).asInt();

                        //given the id, get the value of the property
                        cmd.clear();
                        cmd.addVocab(Vocab::encode("get"));
                        Bottle &content = cmd.addList().addList();
                        Bottle replyProp;
                        content.addString("id");
                        content.addInt(id);
                        opcPort.write(cmd,replyProp);
                        if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                        {
                            if(Bottle *propField = replyProp.get(1).asList())
                            {
                                if(Bottle *subProp=propField->get(0).asList())
                                {
                                    if(Bottle *subPropField=subProp->get(1).asList())
                                    {
                                        if(Bottle *bPose=subPropField->find("pose_camera").asList())
                                        {
                                            est_pose[0]=bPose->get(0).asDouble();
                                            est_pose[1]=bPose->get(1).asDouble();
                                            est_pose[2]=bPose->get(2).asDouble();
                                            return true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    /********************************************************/
    bool updateModule() override
    {
        Vector est_pose(3);
        if(getSkeleton() && getLinePose(est_pose))
        {
            //get skeleton feet
            Vector foot_right=skeletonIn[KeyPointTag::foot_right]->getPoint();
            Vector foot_left=skeletonIn[KeyPointTag::foot_left]->getPoint();
            double dist_fr_line=fabs(est_pose[2]-foot_right[2]);
            double dist_fl_line=fabs(est_pose[2]-foot_left[2]);

            if(dist_fr_line<threshold && dist_fl_line<threshold)
            {
                yInfo() << "Line crossed";
            }

            yInfo() << "foot left:" << foot_left.toString();
            yInfo() << "foot_right:" << foot_right.toString();
            yInfo() << "est_pose:" << est_pose.subVector(0,2).toString();
            yInfo() << "distances:" << dist_fr_line << dist_fl_line;

            Bottle &distscope=scopePort.prepare();
            distscope.clear();
            distscope.addDouble(dist_fr_line);
            distscope.addDouble(dist_fl_line);
            scopePort.write();
        }

        return true;

    }

    bool close() override
    {
        scopePort.close();
        opcPort.close();
        return true;
    }
};

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    CrossingChecker cross;
    return cross.runModule(rf);
}
