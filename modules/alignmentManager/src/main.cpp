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

#include <cstdlib>
#include <fstream>
#include <deque>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include "Dtw.h"
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

class Aligner : public RFModule
{
private:
    RpcServer rpcPort;
    RpcClient opcPort,playerPort;
    BufferedPort<Bottle> port_out, scopePort;

    SkeletonWaist skeletonIn,skeletonTemplate,skeletonAligned;
    string skel_tag,template_tag;
//    Dtw dtw;
    int win;
    bool align;
    vector<Vector> skeleton_template,skeleton_candidate,aligned_skeleton;
    bool updated,start;
    double T,tstart;
    ofstream outfile,outfile2;

public:

//    void getTemplateSkeleton()
//    {
//        //ask for the property id
//        Bottle cmd, reply;
//        cmd.addVocab(Vocab::encode("ask"));
//        Bottle &content = cmd.addList().addList();
//        content.addString("skeleton");

//        opcPort.write(cmd, reply);

//        if(reply.size() > 1)
//        {
//            if(reply.get(0).asVocab() == Vocab::encode("ack"))
//            {
//                if(Bottle *idField = reply.get(1).asList())
//                {
//                    if(Bottle *idValues = idField->get(1).asList())
//                    {
//                        if(!skel_tag.empty())
//                        {
//                            for(int i=0; i<idValues->size(); i++)
//                            {
//                                int id = idValues->get(i).asInt();

//                                //given the id, get the value of the property
//                                cmd.clear();
//                                cmd.addVocab(Vocab::encode("get"));
//                                Bottle &content = cmd.addList().addList();
//                                Bottle replyProp;
//                                content.addString("id");
//                                content.addInt(id);

//                                opcPort.write(cmd, replyProp);
//                                if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
//                                {
//                                    if(Bottle *propField = replyProp.get(1).asList())
//                                    {
//                                        Property prop(propField->toString().c_str());
//                                        string tag=prop.find("tag").asString();
//                                        if(prop.check("tag") && tag==template_tag)
//                                        {
//                                            Skeleton* skeleton = skeleton_factory(prop);
//                                            skeletonTemplate.update(skeleton->toProperty());
//                                            delete skeleton;
//                                        }
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }

    void getSkeleton()
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
                        if(!skel_tag.empty())
                        {
                            updated=false;
                            for(int i=0; i<idValues->size(); i++)
                            {
                                int id = idValues->get(i).asInt();

                                //given the id, get the value of the property
                                cmd.clear();
                                cmd.addVocab(Vocab::encode("get"));
                                Bottle &content = cmd.addList().addList();
                                Bottle replyProp;
                                content.addString("id");
                                content.addInt(id);

                                opcPort.write(cmd, replyProp);
                                if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                                {
                                    if(Bottle *propField = replyProp.get(1).asList())
                                    {
                                        Property prop(propField->toString().c_str());
                                        string tag=prop.find("tag").asString();
                                        if (prop.check("tag") && tag==skel_tag)
                                        {
                                            Skeleton* skeleton = skeleton_factory(prop);
                                            skeletonIn.update(skeleton->toProperty());
                                            updated = true;
                                            delete skeleton;
                                        }
                                        if(prop.check("tag") && tag==template_tag)
                                        {
                                            Skeleton* skeleton = skeleton_factory(prop);
                                            skeletonTemplate.update(skeleton->toProperty());
                                            delete skeleton;
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

    bool is_running()
    {
        Bottle cmd,reply;
        cmd.addString("is_running");
//        yInfo() << cmd.toString();
        playerPort.write(cmd,reply);
        if(reply.get(0).asVocab()==Vocab::encode("ok"))
            return true;
        else
            return false;
    }

    bool loadPlayer()
    {
        Bottle cmd,reply;
        cmd.addString("load");
        cmd.addString(skeletonIn.getTag()+".log");
        cmd.addString("motionAnalyzer");
        playerPort.write(cmd,reply);
        if(reply.get(0).asVocab()==Vocab::encode("ok"))
            return true;
        else
            return false;
    }

    bool startPlayer()
    {
        Bottle cmd,reply;
        cmd.addString("start");
        cmd.addInt(0);
        cmd.addDouble(1.0);
        playerPort.write(cmd,reply);
        if(reply.get(0).asVocab()==Vocab::encode("ok"))
            return true;
        else
            return false;
    }

    bool stopPlayer()
    {
        Bottle cmd,reply;
        cmd.addString("stop");
        playerPort.write(cmd,reply);
        if(reply.get(0).asVocab()==Vocab::encode("ok"))
            return true;
        else
            return false;

    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        if(command.get(0).asString() == "tagt")
        {
            template_tag = command.get(1).asString();

//            //create template
//            double t0 = Time::now();
//            while(is_running())
//            {
//                yInfo() << "Acquiring template...";
//                getTemplateSkeleton();

//                Property prop=skeletonTemplate.toProperty();
//                Bottle &msg=port_out.prepare();
//                msg.clear();
//                msg.addList().read(prop);
//                port_out.write();

//                createTemplate();
//            }
//            T = Time::now()-t0;
//            tstart = Time::now();

//            yInfo() << "Template acquired!" << T;
//            int k=3*skeletonIn.getNumKeyPoints();
//            dtw.initialize(win,skeleton_template.size(),skeleton_template.size(),k);

            reply.addVocab(Vocab::encode("ok"));
        }
        if(command.get(0).asString() == "tag")
        {
            skel_tag = command.get(1).asString();
            reply.addVocab(Vocab::encode("ok"));
        }
        if(command.get(0).asString() == "run")
        {
            int nrep = command.get(1).asInt();
            int nenv = command.get(2).asInt();
            double duration = command.get(3).asDouble();
//            T = nenv*(duration/nrep);

//            stopPlayer();
//            loadPlayer();
//            startPlayer();

            tstart = Time::now();
            start = true;
            reply.addVocab(Vocab::encode("ok"));
        }
        if(command.get(0).asString() == "stop")
        {
            skel_tag.clear();
            template_tag.clear();
            start = false;
            reply.addVocab(Vocab::encode("ok"));
        }
    }

//    void createTemplate()
//    {
//        Vector temp1;
//        temp1.resize(3*skeletonTemplate.getNumKeyPoints());

//        createVec(skeletonTemplate,temp1);
//        skeleton_template.push_back(temp1);
//    }

    bool configure(ResourceFinder &rf) override
    {
        win = rf.check("win",Value(-1)).asDouble();
        T = rf.check("T",Value(1.0)).asDouble();

        outfile.open("dtw-test.txt");
//        outfile2.open("dtw-test2.txt");

        opcPort.open("/alignmentManager/opc");
        playerPort.open("/alignmentManager/player");
        port_out.open("/alignmentManager");
        scopePort.open("/alignmentManager/scope");
        rpcPort.open("/alignmentManager/rpc");
        attach(rpcPort);

        start = false;

        return true;
    }

    bool interruptModule() override
    {
        opcPort.interrupt();
        playerPort.interrupt();
        port_out.interrupt();
        scopePort.interrupt();
        rpcPort.interrupt();
        yInfo() << "Interrupted module";
        return true;
    }

    bool close() override
    {
        outfile.close();
//        outfile2.close();

        opcPort.close();
        playerPort.close();
        port_out.close();
        scopePort.close();
        rpcPort.close();
        yInfo() << "Closed ports";
        return true;
    }

    double getPeriod() override
    {
        return 1.0;
    }

    void createVec(const SkeletonWaist& skeleton, Vector& temp)
    {
        temp[0] = skeleton[KeyPointTag::shoulder_center]->getPoint()[0];
        temp[1] = skeleton[KeyPointTag::shoulder_center]->getPoint()[1];
        temp[2] = 0.0; //skeleton[KeyPointTag::shoulder_center]->getPoint()[2];

        temp[3] = skeleton[KeyPointTag::head]->getPoint()[0];
        temp[4] = skeleton[KeyPointTag::head]->getPoint()[1];
        temp[5] = 0.0; //skeleton[KeyPointTag::head]->getPoint()[2];

        temp[6] = skeleton[KeyPointTag::shoulder_left]->getPoint()[0];
        temp[7] = skeleton[KeyPointTag::shoulder_left]->getPoint()[1];
        temp[8] = 0.0; //skeleton[KeyPointTag::shoulder_left]->getPoint()[2];

        temp[9] = skeleton[KeyPointTag::elbow_left]->getPoint()[0];
        temp[10] = skeleton[KeyPointTag::elbow_left]->getPoint()[1];
        temp[11] = 0.0; //skeleton[KeyPointTag::elbow_left]->getPoint()[2];

        temp[12] = skeleton[KeyPointTag::hand_left]->getPoint()[0];
        temp[13] = skeleton[KeyPointTag::hand_left]->getPoint()[1];
        temp[14] = 0.0; //skeleton[KeyPointTag::hand_left]->getPoint()[2];

        temp[15] = skeleton[KeyPointTag::shoulder_right]->getPoint()[0];
        temp[16] = skeleton[KeyPointTag::shoulder_right]->getPoint()[1];
        temp[17] = 0.0; //skeleton[KeyPointTag::shoulder_right]->getPoint()[2];

        temp[18] = skeleton[KeyPointTag::elbow_right]->getPoint()[0];
        temp[19] = skeleton[KeyPointTag::elbow_right]->getPoint()[1];
        temp[20] = 0.0; //skeleton[KeyPointTag::elbow_right]->getPoint()[2];

        temp[21] = skeleton[KeyPointTag::hand_right]->getPoint()[0];
        temp[22] = skeleton[KeyPointTag::hand_right]->getPoint()[1];
        temp[23] = 0.0; //skeleton[KeyPointTag::hand_right]->getPoint()[2];

        temp[24] = skeleton[KeyPointTag::hip_left]->getPoint()[0];
        temp[25] = skeleton[KeyPointTag::hip_left]->getPoint()[1];
        temp[26] = 0.0; //skeleton[KeyPointTag::hip_left]->getPoint()[2];

        temp[27] = skeleton[KeyPointTag::hip_center]->getPoint()[0];
        temp[28] = skeleton[KeyPointTag::hip_center]->getPoint()[1];
        temp[29] = 0.0; //skeleton[KeyPointTag::hip_center]->getPoint()[2];

        temp[30] = skeleton[KeyPointTag::knee_left]->getPoint()[0];
        temp[31] = skeleton[KeyPointTag::knee_left]->getPoint()[1];
        temp[32] = 0.0; //skeleton[KeyPointTag::knee_left]->getPoint()[2];

        temp[33] = skeleton[KeyPointTag::ankle_left]->getPoint()[0];
        temp[34] = skeleton[KeyPointTag::ankle_left]->getPoint()[1];
        temp[35] = 0.0; //skeleton[KeyPointTag::ankle_left]->getPoint()[2];

        temp[36] = skeleton[KeyPointTag::hip_right]->getPoint()[0];
        temp[37] = skeleton[KeyPointTag::hip_right]->getPoint()[1];
        temp[38] = 0.0; //skeleton[KeyPointTag::hip_right]->getPoint()[2];

        temp[39] = skeleton[KeyPointTag::knee_right]->getPoint()[0];
        temp[40] = skeleton[KeyPointTag::knee_right]->getPoint()[1];
        temp[41] = 0.0; //skeleton[KeyPointTag::knee_right]->getPoint()[2];

        temp[42] = skeleton[KeyPointTag::ankle_right]->getPoint()[0];
        temp[43] = skeleton[KeyPointTag::ankle_right]->getPoint()[1];
        temp[44] = 0.0; //skeleton[KeyPointTag::ankle_right]->getPoint()[2];
    }

    void updateVec()
    {
        Vector temp1;
        temp1.resize(3*skeletonTemplate.getNumKeyPoints());

        createVec(skeletonTemplate,temp1);
        skeleton_template.push_back(temp1);

        Vector temp2;
        temp2.resize(3*skeletonIn.getNumKeyPoints());

        createVec(skeletonIn,temp2);
        skeleton_candidate.push_back(temp2);

//        if(skeleton_candidate.size()>skeleton_template.size())
//            skeleton_candidate.erase(skeleton_candidate.begin());
    }

    bool updateModule() override
    {
        //if we query the database
        if(opcPort.getOutputCount() > 0 && start)
        {
            //get skeleton
            getSkeleton();

            if(updated)
            {
                //update vectors to align
                updateVec();

                if( (Time::now()-tstart)> T )
                {
                    Dtw *dtw;

                    dtw=new Dtw(win,skeleton_template.size(),skeleton_candidate.size());

                    double d=0.0;
                    vector<double> s_template,s_candidate;
                    for(int i=0;i<3*skeletonIn.getNumKeyPoints();i++)
                    {
                        for(int j=0;j<skeleton_template.size();j++)
                        {
                            s_template.push_back(skeleton_template[j][i]);
                            s_candidate.push_back(skeleton_candidate[j][i]);
                        }
                        d+=dtw->computeDistance(s_template,s_candidate);

//                        for(int i=0; i<s_template.size(); i++)
//                            outfile2 << s_template[i] << " " << s_candidate[i] << "\n";
//                        outfile2 << "\n";

                        s_template.clear();
                        s_candidate.clear();
                    }
                    yInfo() << T << d;



//                    for(int i=0; i<skeleton_template.size(); i++)
//                        outfile << skeleton_template[i][0] << " " << skeleton_candidate[i][0] << "\n";

//                    for(int i=0; i<skeleton_template.size(); i++)
//                    {
//                        for(int j=0; j<3*skeletonTemplate.getNumKeyPoints(); j++)
//                        {
//                            outfile << skeleton_template[i][j] << " ";
//                        }

//                        for(int j=0; j<3*skeletonIn.getNumKeyPoints(); j++)
//                        {
//                            outfile << skeleton_candidate[i][j] << " ";
//                        }
//                        outfile << "\n";
//                    }
//                    outfile << "\n";

                    tstart=Time::now();
                    skeleton_template.clear();
                    skeleton_candidate.clear();

                    delete dtw;
                }

//                dtw.update(skeleton_template,skeleton_candidate);
//                aligned_skeleton=dtw.align();
//                vector<pair<string,Vector>> unordered;
//                Vector p=aligned_skeleton.back();
//                updateUnordered(unordered,p);

            }
        }

        return true;
    }

    void updateUnordered(vector<pair<string,Vector>>& unordered, const Vector& p)
    {
        if(skeletonIn[KeyPointTag::shoulder_center]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::shoulder_center,p.subVector(0,2)));

        if(skeletonIn[KeyPointTag::head]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::head,p.subVector(3,5)));

        if(skeletonIn[KeyPointTag::shoulder_left]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::shoulder_left,p.subVector(6,8)));

        if(skeletonIn[KeyPointTag::elbow_left]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::elbow_left,p.subVector(9,11)));

        if(skeletonIn[KeyPointTag::hand_left]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::hand_left,p.subVector(12,14)));

        if(skeletonIn[KeyPointTag::shoulder_right]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::shoulder_right,p.subVector(15,17)));

        if(skeletonIn[KeyPointTag::elbow_right]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::elbow_right,p.subVector(18,20)));

        if(skeletonIn[KeyPointTag::hand_right]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::hand_right,p.subVector(21,23)));

        if(skeletonIn[KeyPointTag::hip_left]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::hip_left,p.subVector(24,26)));

        if(skeletonIn[KeyPointTag::hip_center]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::hip_center,p.subVector(27,29)));

        if(skeletonIn[KeyPointTag::knee_left]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::knee_left,p.subVector(30,32)));

        if(skeletonIn[KeyPointTag::ankle_left]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::ankle_left,p.subVector(33,35)));

        if(skeletonIn[KeyPointTag::hip_right]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::hip_right,p.subVector(36,38)));

        if(skeletonIn[KeyPointTag::knee_right]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::knee_right,p.subVector(39,41)));

        if(skeletonIn[KeyPointTag::ankle_right]->isUpdated())
            unordered.push_back(make_pair(KeyPointTag::ankle_right,p.subVector(42,44)));
    }
};

int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Aligner aligner;

    return aligner.runModule(rf);
}
