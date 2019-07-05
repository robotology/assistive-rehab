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

#include <iostream>
#include <string>
#include <cmath>

#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

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
    RpcServer rpcPort;

    string file;
//    string context;
    double opacity;
    int nsessions;
    double tbegin;

    bool hasStarted;

    string sel_tag;
    string prev_tag;

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        opcPort.open("/skeletonScaler/opc");
        cmdPort.open("/skeletonScaler/player:rpc");
        rpcViewerPort.open("/skeletonScaler/viewer:rpc");
        rpcPort.open("/skeletonScaler/rpc");
        attach(rpcPort);

        nsessions=rf.check("nsessions",Value(0)).asInt();
        tbegin=rf.check("tbegin",Value(0.0)).asDouble();

        hasStarted=false;
        sel_tag="";
        prev_tag="";

        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.01;
    }

    /****************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        if(command.get(0).asString() == "load")
        {
            file = command.get(1).asString();
            string context = command.get(2).asString();
            if(loadData(file,context))
            {
                reply.addVocab(Vocab::encode("ok"));
                reply.addString("Loading file " + file + " from context " + context);

                size_t idx=file.find(".");
                setTag(file.substr(0,idx));

                opacity=0.3;
                setOpacity(opacity);
            }
            else
            {
                yError() << "Unable to find" << file;
                return false;
            }
        }
        if(command.get(0).asString() == "rot")
        {
            Vector camerapos(3,0.0),focalpoint(3,0.0);
            Bottle *cp = command.get(1).asList();
            Bottle *fp = command.get(2).asList();
            camerapos[0] = cp->get(0).asDouble();
            camerapos[1] = cp->get(1).asDouble();
            camerapos[2] = cp->get(2).asDouble();

            focalpoint[0] = fp->get(0).asDouble();
            focalpoint[1] = fp->get(1).asDouble();
            focalpoint[2] = fp->get(2).asDouble();

            if(rotateCam(camerapos,focalpoint))
            {
                reply.addVocab(Vocab::encode("ok"));
            }
            else
            {
                reply.addVocab(Vocab::encode("fail"));
                yWarning() << "Unable to rotate camera";
            }
        }
        if(command.get(0).asString() == "run")
        {
            double twarp = command.get(1).asDouble();
            if(start(nsessions,twarp))
            {
                reply.addVocab(Vocab::encode("ok"));
                hasStarted=true;
            }
            else
            {
                yError() << "Unable to start";
                return false;
            }
        }
        if(command.get(0).asString() == "tags")
        {
            selectByTag(command.get(1).asString());
            reply.addVocab(Vocab::encode("ok"));
        }
        if(command.get(0).asString() == "stop")
        {
            if(stop())
            {
                hide();
                hasStarted=false;
                reply.addVocab(Vocab::encode("ok"));
            }
            else
            {
                yError() << "Unable to stop";
                return false;
            }
        }

        return true;
    }

    /****************************************************************/
    bool updateModule()
    {
        return true;
    }

    /****************************************************************/
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

    /****************************************************************/
    bool isZero(const Matrix& m)
    {
        int count=0;
        for(int i=0;i<m.rows();i++)
        {
            for(int j=0;j<m.cols();j++)
            {
                if(m[i][j]!=0)
                    count++;
            }
        }
        return (count < 1);

    }

    /****************************************************************/
    void getSkeletonsFromOpc(SkeletonStd& skeleton, SkeletonStd& playedSkel)
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
                        bool hasUpdated=false;
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
                                        if(prop.check("tag"))
                                        {
                                            if(file.find(tag)==string::npos)
                                            {
                                                if(!sel_tag.empty() && tag==sel_tag)
                                                {
                                                    Skeleton* skel1 = skeleton_factory(prop);
                                                    skeleton.update(skel1->toProperty());
                                                    delete skel1;
                                                }
                                            }
                                            else
                                            {
                                                Skeleton* skel2 = skeleton_factory(prop);
                                                playedSkel.update(skel2->toProperty());
                                                delete skel2;
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
        Bottle &content1 = cmd.addList();
        content1.addString("position");
        Bottle &position = content1.addList();
        position.addDouble(camerapos[0]);
        position.addDouble(camerapos[1]);
        position.addDouble(camerapos[2]);
        Bottle &content2 = cmd.addList();
        content2.addString("focalpoint");
        Bottle &fp = content2.addList();
        fp.addDouble(focalpoint[0]);
        fp.addDouble(focalpoint[1]);
        fp.addDouble(focalpoint[2]);

        yInfo() << cmd.toString();
        if(rpcViewerPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ack"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool moveSkeleton(const Matrix& T)
    {
        Bottle cmd,rep;
        cmd.addString("move");
        cmd.addList().read(T);
        yInfo() << cmd.toString();
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    void getMaxPath(double& maxpath,const double& tbegin_)
    {
        Bottle cmd,rep;
        cmd.addString("get_maxpath");
        cmd.addDouble(tbegin_);
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
        yInfo() << cmd.toString();
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
            {
                return true;
            }
        }

        return false;
    }

    /****************************************************************/
    bool start(const int& nsessions_, const double& twarp_)
    {
        Bottle cmd1,rep1;
        cmd1.addString("put_in_opc");
        cmd1.addDouble(0.0);
        yInfo() << cmd1.toString();
        if(cmdPort.write(cmd1,rep1))
        {
            if(!rep1.get(0).asVocab()==Vocab::encode("ok"))
                return false;
        }

        SkeletonStd retrievedSkel, playedSkel;

        getSkeletonsFromOpc(retrievedSkel,playedSkel);
        yInfo() << retrievedSkel.getTag() << playedSkel.getTag();

        Matrix T;
        if(!retrievedSkel.getTag().empty())
        {
            Vector p1=retrievedSkel[KeyPointTag::shoulder_center]->getPoint();
            Vector c1=retrievedSkel.getCoronal();
            Vector t1=retrievedSkel.getTransverse();
            Vector s1=retrievedSkel.getSagittal();
            Matrix Temp1 =zeros(4,4);
            Temp1.setSubcol(c1,0,0);
            Temp1.setSubcol(s1,0,1);
            Temp1.setSubcol(t1,0,2);
            Temp1.setSubcol(p1,0,3);
            Temp1(3,3)=1.0;

            Vector p2=playedSkel[KeyPointTag::shoulder_center]->getPoint();
            Vector c2=playedSkel.getCoronal();
            Vector t2=playedSkel.getTransverse();
            Vector s2=playedSkel.getSagittal();
            Matrix Temp2 =zeros(4,4);
            Temp2.setSubcol(c2,0,0);
            Temp2.setSubcol(s2,0,1);
            Temp2.setSubcol(t2,0,2);
            Temp2.setSubcol(p2,0,3);
            Temp2(3,3)=1.0;

            T = Temp1*SE3inv(Temp2);
        }

//        double maxpath;
//        getMaxPath(maxpath,tbegin);
//        double scale=retrievedSkel.getMaxPath()/maxpath;
//        if(!setScale(scale))
//            yWarning() << "Unable to scale";
                        
        if(!moveSkeleton(T))
            yWarning() << "Unable to move";        

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
    bool stop()
    {
        Bottle cmd,rep;
        cmd.addString("stop");
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
            {
                yInfo() << "Stopping";
                prev_tag="";
                return true;
            }
        }

        return false;
    }

    /****************************************************************/
    bool hide()
    {
        Bottle cmd,rep;
        cmd.addString("remove_from_opc");
        cmdPort.write(cmd,rep);
        if(rep.get(0).asVocab()==Vocab::encode("ok"))
            return true;
        return false;
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
    void selectByTag(const string& tag)
    {
        Bottle cmd,rep;
        cmd.addString("tag");
        cmd.addString(tag);
        yInfo() << cmd.toString();
        cmdPort.write(cmd,rep);
        sel_tag=tag;
    }

    /****************************************************************/
    bool interruptModule()
    {
       stop();
       hide();
       opcPort.interrupt();
       cmdPort.interrupt();
       rpcViewerPort.interrupt();

       return true;
    }

    /****************************************************************/
    bool close()
    {
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

