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
    double twarp;

    bool hasStarted;

    Matrix invT;
//    Vector rot;
//    Vector xyz;
    Vector xyz_prev;

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

//        if(!Network::connect(cmdPort.getName(),"/skeletonPlayer/cmd:rpc", "tcp"))
//        {
//            yError() << "Cannot connect to /skeletonPlayer/cmd:rpc";
//            return false;
//        }

//        if(!Network::connect(rpcViewerPort.getName(),"/skeletonViewer:rpc", "tcp"))
//        {
//            yError() << "Cannot connect to /skeletonViewer:rpc";
//            return false;
//        }

//        file=rf.check("file",Value("abduction.log")).asString();
//        context=rf.check("context",Value("motionAnalyzer")).asString();

        nsessions=rf.check("nsessions",Value(0)).asInt();
        twarp=rf.check("twarp",Value(0.5)).asDouble();

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

//                xyz.resize(3);
//                xyz.zero();
//                xyz_prev.resize(3);
//                xyz_prev.zero();
//                rot.resize(4);
//                rot.zero();
                if(file.find("abduction")!=string::npos)
                {
//                    xyz[0]=-0.12;
//                    xyz[1]=0.24;
//                    xyz[2]=-1.75;
                    Vector camerapos(3,0.0),focalpoint(3,0.0);
                    camerapos[2]=-2.0;
                    rotateCam(camerapos,focalpoint);
                }
                if(file.find("flexion")!=string::npos)
                {
//                    xyz[0]=-0.12;
//                    xyz[1]=0.24;
//                    xyz[2]=-1.75;
                    Vector camerapos(3,0.0),focalpoint(3,0.0);
                    camerapos[0]=4.0;
                    focalpoint[2]=1.0;
                    rotateCam(camerapos,focalpoint);
                }
//                if(!moveSkeleton(xyz,rot))
//                    yWarning() << "Unable to move";

/*                Matrix T=axis2dcm(rot);
                T(0,3)=xyz[0];
                T(1,3)=xyz[1];
                T(2,3)=xyz[2];
                invT=SE3inv(T);
*/                
//                invT.resize(4,4);
//                invT.zero();

            }
        }
        if(command.get(0).asString() == "run")
        {
            if(start(nsessions,twarp))
            {
                reply.addVocab(Vocab::encode("ok"));
                hasStarted=true;
            }
        }
        if(command.get(0).asString() == "tag")
        {
            selectByTag(command.get(1).asString());
            reply.addString("Selecting skeleton" + command.get(1).asString());
        }
        if(command.get(0).asString() == "stop")
        {
            if(stop())
            {
                hide();
                hasStarted=false;
                reply.addVocab(Vocab::encode("ok"));
            }
        }

        return true;
    }

    /****************************************************************/
    bool updateModule()
    {
 //       if(opcPort.getOutputCount()>0 && hasStarted)
//        {
//            SkeletonWaist retrievedSkel, playedSkel;
//            getSkeletonsFromOpc(retrievedSkel,playedSkel);

//            if(!retrievedSkel.getTag().empty() && retrievedSkel.getTag()!="#8c"
//                    && retrievedSkel.getTag()!="flexion" && retrievedSkel.getTag()!="abduction")
//            {
//                yInfo() << retrievedSkel.getTag();
//                if(retrievedSkel.getTag()==prev_tag || prev_tag.empty())
//                {
//                    xyz=retrievedSkel[KeyPointTag::shoulder_center]->getPoint();
//                    Vector p1=retrievedSkel.getCoronal();
//                    Vector p2=playedSkel.getCoronal();
//                    Vector axis=cross(p2,p1);
//                    if(norm(axis)>0.0)
//                        axis/=norm(axis);
//                    double angle=acos(dot(p2,p1)/norm(p1)*norm(p2));
//                    rot[0]=axis[0];
//                    rot[1]=axis[1];
//                    rot[2]=axis[2];
//                    rot[3]=angle;

//                    Matrix T=axis2dcm(rot);
//                    T(0,3)=xyz[0];
//                    T(1,3)=xyz[1];
//                    T(2,3)=xyz[2];

                    //current transformation
//                    Matrix currT;
//                    Vector curr_rot,xyz_inv(3,0.0);
//                    curr_rot.resize(3);
//                    if(!isZero(invT))
//                    {
//                        currT=multiply(T,invT.transposed());
//                        curr_rot=dcm2axis(currT);

//                        Matrix prevRot(4,4);
//                        prevRot.setSubmatrix(invT.submatrix(0,2,0,2).transposed(),0,0);
//                        Vector v(4,0.0);
//                        v[3]=1.0;
//                        prevRot.setSubrow(v,3,3);
//                        prevRot.setSubcol(v,3,3);
//                        xyz_inv=(prevRot*invT).subcol(0,3,3);
//                    }
//                    else
//                        curr_rot=rot;

//                    double n=norm(xyz_prev-xyz);
//                    xyz_prev=xyz;
//                    if(n<pow(10.0,-5.0))
//                        curr_rot.zero();

//                    xyz+=xyz_inv;
//
//                    if(!moveSkeleton(xyz,curr_rot))
//                        yWarning() << "Unable to move";

//                    invT=SE3inv(T);

//                    //retrieve previous translation
//                    Matrix prevRot(4,4);
//                    prevRot.setSubmatrix(invT.submatrix(0,2,0,2).transposed(),0,0);
//                    Vector v(4,0.0);
//                    v[3]=1;
//                    prevRot.setSubrow(v,3,3);
//                    prevRot.setSubcol(v,3,3);
//                    xyz_inv=(prevRot*invT).subcol(0,3,3);

//                    Vector rot2;
//                    if(!isZero(invT))
//                    {
//                        Matrix rotback=multiply(T.submatrix(0,2,0,2),(invT.submatrix(0,2,0,2)).transposed());
//                        rot2=dcm2axis(rotback);
//                    }
//                    else
//                        rot2=rot;

////                    double n=norm(xyz_prev-xyz);
////                    xyz_prev=xyz;
////                    if(n<pow(10.0,-6.0))
////                        rot2.zero();

//                    xyz[0]+=xyz_inv[0];
//                    xyz[1]+=xyz_inv[1];
//                    xyz[2]+=xyz_inv[2];

//                    if(!moveSkeleton(xyz,rot2))
//                        yWarning() << "Unable to move";

//                    //retrieve transformation from unmoved skeleton
//                    invT=SE3inv(T);

//                    double maxpath;
//                    getMaxPath(maxpath);
//                    double scale=retrievedSkel.getMaxPath()/maxpath;
//                    if(!setScale(scale))
//                    {
//                        yError() << "Unable to scale";
//                        return false;
//                    }

//                    prev_tag=retrievedSkel.getTag();
//                }
//                else
//                {
//                    stop();
//                    hide();

                    //invert current transformation to go back to the center
//                    xyz.zero();

                    //get axis-angle to align coronal planes
/*                    Vector p1(3,0.0);
                    p1[2]=-1.0;
                    Vector p2=playedSkel.getCoronal();
                    Vector axis=cross(p2,p1);
                    if(norm(axis)>0.0)
                        axis/=norm(axis);
                    double angle=acos(dot(p2,p1)/norm(p1)*norm(p2));
                    rot[0]=axis[0];
                    rot[1]=axis[1];
                    rot[2]=axis[2];
                    rot[3]=angle;

                    Matrix T=axis2dcm(rot);
                    T(0,3)=xyz[0];
                    T(1,3)=xyz[1];
                    T(2,3)=xyz[2];

                    //current transformation
                    Matrix currT;
                    Vector curr_rot,xyz_inv(3,0.0);
                    curr_rot.resize(3);
                    if(!isZero(invT))
                    {
                        currT=multiply(T,invT.transposed());
                        curr_rot=dcm2axis(currT);

                        Matrix prevRot(4,4);
                        prevRot.setSubmatrix(invT.submatrix(0,2,0,2).transposed(),0,0);
                        Vector v(4,0.0);
                        v[3]=1.0;
                        prevRot.setSubrow(v,3,3);
                        prevRot.setSubcol(v,3,3);
                        xyz_inv=(prevRot*invT).subcol(0,3,3);
                    }
                    else
                        curr_rot=rot;
                    xyz+=xyz_inv;

                    if(!moveSkeleton(xyz,curr_rot))
                        yWarning() << "Unable to move";

                    invT.zero();

                }
            }
        } */

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
    Matrix multiply(const Matrix& m1, const Matrix& m2)
    {
        Matrix res(m1.rows(),m1.cols());
        for(int i=0;i<m1.rows();i++)
        {
            for(int j=0;j<m1.cols();j++)
            {
                res[i][j]=m1[i][j]*m2[i][j];
            }
        }
        return res;
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
    void getSkeletonsFromOpc(SkeletonWaist& skeleton, SkeletonWaist& playedSkel)
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
                                                    skeleton.update_fromstd(skel1->toProperty());
                                                    delete skel1;
                                                }
                                            }
                                            else
                                            {
                                                Skeleton* skel2 = skeleton_factory(prop);
                                                playedSkel.update_fromstd(skel2->toProperty());
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
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
                return true;
        }

        return false;
    }

    /****************************************************************/
    bool moveSkeleton(const Matrix& T)
    {
        Bottle cmd,payload,rep;
        payload.read(const_cast<Matrix&>(T));
        cmd.addString("move");
        cmd.append(payload);
        yInfo() << cmd.toString();
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

        SkeletonWaist retrievedSkel, playedSkel;

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

        double maxpath;
        getMaxPath(maxpath);
        double scale=retrievedSkel.getMaxPath()/maxpath;
        if(!setScale(scale))
            yWarning() << "Unable to scale";
                        
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
//        xyz.zero();
        Bottle cmd,rep;
        cmd.addString("stop");
        if(cmdPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
            {
                yInfo() << "Stopping";
                prev_tag="";

                if(file.find("flexion")!=string::npos)
                {
                    Vector camerapos(3,0.0),focalpoint(3,0.0);
                    camerapos[2]=-4.0;
                    rotateCam(camerapos,focalpoint);
                }

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

