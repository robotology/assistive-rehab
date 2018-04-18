/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include "string"
#include "cmath"

#include "yarp/sig/Vector.h"
#include "yarp/os/all.h"
#include "yarp/math/Math.h"

#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class Scaler : public RFModule
{
    RpcClient opcPort;
    RpcClient cmdPort;
    RpcClient rpcViewerPort;

    string file;
    string context;
    double opacity;
    int nsessions;
    double twarp;

    Matrix invT;
    Vector rot;
    Vector xyz;

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        opcPort.open("/skeletonScaler/opc");
        cmdPort.open("/skeletonScaler/player:rpc");
        rpcViewerPort.open("/skeletonScaler/viewer:rpc");

        if(!Network::connect(cmdPort.getName(),"/skeletonPlayer/cmd:rpc", "tcp"))
        {
            yError() << "Cannot connect to /skeletonPlayer/cmd:rpc";
            return false;
        }

        if(!Network::connect(rpcViewerPort.getName(),"/skeletonViewer:rpc", "tcp"))
        {
            yError() << "Cannot connect to /skeletonViewer:rpc";
            return false;
        }

        file=rf.check("file",Value("abduction.log")).asString();
        context=rf.check("context",Value("motionAnalyzer")).asString();

        nsessions=rf.check("nsessions",Value(0)).asInt();
        twarp=rf.check("twarp",Value(0.5)).asDouble();

        if(!loadData(file,context))
        {
            yError() << "Unable to load data";
            return false;
        }

        if(!start(nsessions,twarp))
        {
            yError() << "Unable to start";
            return false;
        }

        int idx=file.find(".");
        setTag(file.substr(0,idx));

        opacity=0.1;
        setOpacity(opacity);

        xyz.resize(3);
        xyz.zero();
        rot.resize(4);
        rot.zero();
        if(file.find("abduction")!=string::npos)
        {
            xyz[0]=-0.12;
            xyz[1]=0.24;
            xyz[2]=-1.75;
            Vector camerapos(3,0.0),focalpoint(3,0.0);
            camerapos[2]=-2.0;
            rotateCam(camerapos,focalpoint);
        }
        if(file.find("flexion")!=string::npos)
        {
            xyz[0]=-0.12;
            xyz[1]=0.24;
            xyz[2]=-1.75;
            Vector camerapos(3,0.0),focalpoint(3,0.0);
            camerapos[0]=4.0;
            rotateCam(camerapos,focalpoint);
        }
        if(!moveSkeleton(xyz,rot))
            yWarning() << "Unable to move";

        invT.resize(4,4);
        invT.zero();

        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.01;
    }

    /****************************************************************/
    bool updateModule()
    {
        if(opcPort.getOutputCount()>0)
        {
            SkeletonWaist retrievedSkel;
            getSkeletonFromOpc(retrievedSkel);
//            retrievedSkel.print();

            Vector xyz_inv=invT.subcol(0,3,3);
            xyz=retrievedSkel[KeyPointTag::shoulder_center]->getPoint();

            //retrieve transformation from unmoved skeleton
            Matrix T=axis2dcm(rot);
            T(0,3)=xyz[0];
            T(1,3)=xyz[1];
            T(2,3)=xyz[2];
            invT=SE3inv(T);

            xyz[0]+=xyz_inv[0];
            xyz[1]+=xyz_inv[1];
            xyz[2]+=xyz_inv[2];
            if(!moveSkeleton(xyz,rot))
                yWarning() << "Unable to move";

            double maxpath;
            if(!retrievedSkel.getTag().empty())
            {
//                yInfo() << retrievedSkel.getTag().c_str();
                getMaxPath(maxpath);
                double scale=retrievedSkel.getMaxPath()/maxpath;
                if(!setScale(scale))
                {
                    yError() << "Unable to scale";
                    return false;
                }
            }
        }

        return true;
    }

    /****************************************************************/
    void getSkeletonFromOpc(SkeletonWaist& skeleton)
    {
        //ask for the property id
        Bottle cmd, reply;
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content = cmd.addList().addList();
        content.addString("skeleton");
        opcPort.write(cmd, reply);

        if(reply.size() > 1)
        {
            if(reply.get(0).asVocab() == Vocab::encode("ack"))
            {
                if(Bottle *idField = reply.get(1).asList())
                {
                    if(Bottle *idValues = idField->get(1).asList())
                    {
                        for(int i=0; i<idValues->size(); i++)
                        {
                            int id = idValues->get(i).asInt();

                            //given the id, get the value of the property
                            cmd.clear();
                            cmd.addVocab(Vocab::encode("get"));
                            Bottle &content = cmd.addList().addList();
                            content.addString("id");
                            content.addInt(id);
                            Bottle replyProp;
                            opcPort.write(cmd, replyProp);

                            if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                            {
                                if(Bottle *propField = replyProp.get(1).asList())
                                {
                                    Property prop(propField->toString().c_str());
                                    string tag=prop.find("tag").asString();
                                    if(!tag.empty())
                                    {
                                        if(file.find(tag)==string::npos)
                                        {
                                            if(prop.check("tag"))
                                            {
                                                Skeleton* skel = skeleton_factory(prop);
                                                skeleton.update_fromstd(skel->toProperty());
//                                                skeleton.print();
                                                delete skel;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /****************************************************************/
    bool rotateCam(const Vector& camerapos,const Vector& focalpoint)
    {
        Bottle cmd,rep;
        cmd.addString("set_camera");
        Bottle &content = cmd.addList();
        content.addString("position");
        Bottle &position = content.addList();
        position.addDouble(camerapos[0]);
        position.addDouble(camerapos[1]);
        position.addDouble(camerapos[2]);
        content.addString("focalpoint");
        Bottle &fp = content.addList();
        fp.addDouble(focalpoint[0]);
        fp.addDouble(focalpoint[1]);
        fp.addDouble(focalpoint[2]);

        yInfo() << cmd.toString();
        if(rpcViewerPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool moveSkeleton(const Vector& xyz,const Vector& rot)
    {
        Bottle cmd,rep;
        cmd.addString("move");
        cmd.addDouble(xyz[0]);
        cmd.addDouble(xyz[1]);
        cmd.addDouble(xyz[2]);
        cmd.addDouble(rot[0]);
        cmd.addDouble(rot[1]);
        cmd.addDouble(rot[2]);
        cmd.addDouble(rot[3]);
//        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    void getMaxPath(double& maxpath)
    {
        Bottle cmd,rep;
        cmd.addString("get_maxpath");
//        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
            maxpath=rep.get(0).asDouble();
    }

    /****************************************************************/
    bool setScale(const double& scale)
    {
        Bottle cmd,rep;
        cmd.addString("scale");
        cmd.addDouble(scale);
//        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool setOpacity(const double& opacity_)
    {
        Bottle cmd,rep;
        cmd.addString("set_opacity");
        cmd.addDouble(opacity_);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool loadData(const string& file_, const string& context_)
    {
        Bottle cmd,rep;
        cmd.addString("load");
        cmd.addString(file_);
        cmd.addString(context_);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool start(const int& nsessions_, const double& twarp_)
    {
        Bottle cmd,rep;
        cmd.addString("start");
        cmd.addInt(nsessions_);
        cmd.addDouble(twarp_);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    void stop()
    {
        Bottle cmd,rep;
        cmd.addString("stop");
        yInfo() << cmd.toString();
        cmdPort.write(cmd,rep);
    }

    /****************************************************************/
    void setTag(const string& tag)
    {
        Bottle cmd,rep;
        cmd.addString("set_tag");
        cmd.addString(tag);
        yInfo() << cmd.toString();
        cmdPort.write(cmd,rep);
    }

    /****************************************************************/
    bool interruptModule()
    {
       stop();
//       retrieverPort.interrupt();
       opcPort.interrupt();
       cmdPort.interrupt();
       rpcViewerPort.interrupt();

       return true;
    }

    /****************************************************************/
    bool close()
    {
//        retrieverPort.close();
        opcPort.close();
        cmdPort.close();
        rpcViewerPort.close();

        return true;
    }

};

/****************************************************************/
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

    Scaler scaler;
    return scaler.runModule(rf);
}

