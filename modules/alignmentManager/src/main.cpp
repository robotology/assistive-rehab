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

#include <algorithm>
#include <fstream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include "Dtw.h"
#include <fftw3.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

class Aligner : public RFModule
{
private:
    //ports
    RpcServer rpcPort;
    RpcClient opcPort;
    BufferedPort<Bottle> port_out,scopePort;

    //parameters
    int win;
    double T,tstart,dtw_thresh,period,errpos_thresh;

    SkeletonWaist skeletonIn,skeletonTemplate;
    string skel_tag,template_tag;
    vector<vector<Vector>> skeleton_template,skeleton_candidate;
    bool updated,start;
    vector<double> s_template,s_candidate,s_aligned;
    ofstream outfile;

public:

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

    bool respond(const Bottle &command, Bottle &reply)
    {
        if(command.get(0).asString() == "tagt")
        {
            template_tag = command.get(1).asString();
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
            T = nenv*(duration/nrep);

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

    bool configure(ResourceFinder &rf) override
    {
        win = rf.check("win",Value(-1)).asDouble();
//        T = rf.check("T",Value(1.0)).asDouble();
        period = rf.check("period",Value(0.01)).asDouble();
        dtw_thresh = rf.check("threshold",Value(0.10)).asDouble();
        errpos_thresh = rf.check("errpos_thresh",Value(0.40)).asDouble();

        outfile.open("dtw-test.txt");

        opcPort.open("/alignmentManager/opc");
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
        port_out.interrupt();
        scopePort.interrupt();
        rpcPort.interrupt();
        yInfo() << "Interrupted module";
        return true;
    }

    bool close() override
    {
        outfile.close();

        opcPort.close();
        port_out.close();
        scopePort.close();
        rpcPort.close();
        yInfo() << "Closed ports";
        return true;
    }

    double getPeriod() override
    {
        return period;
    }

    void createVec(const SkeletonWaist& skeleton, vector<Vector>& temp)
    {       
        //same order as in skeletonWaist
        temp.push_back(skeleton[KeyPointTag::shoulder_center]->getPoint());
        temp.push_back(skeleton[KeyPointTag::head]->getPoint());
        temp.push_back(skeleton[KeyPointTag::shoulder_left]->getPoint());
        temp.push_back(skeleton[KeyPointTag::elbow_left]->getPoint());
        temp.push_back(skeleton[KeyPointTag::hand_left]->getPoint());
        temp.push_back(skeleton[KeyPointTag::shoulder_right]->getPoint());
        temp.push_back(skeleton[KeyPointTag::elbow_right]->getPoint());
        temp.push_back(skeleton[KeyPointTag::hand_right]->getPoint());
        temp.push_back(skeleton[KeyPointTag::hip_center]->getPoint());
        temp.push_back(skeleton[KeyPointTag::hip_left]->getPoint());
        temp.push_back(skeleton[KeyPointTag::knee_left]->getPoint());
        temp.push_back(skeleton[KeyPointTag::ankle_left]->getPoint());
        temp.push_back(skeleton[KeyPointTag::hip_right]->getPoint());
        temp.push_back(skeleton[KeyPointTag::knee_right]->getPoint());
        temp.push_back(skeleton[KeyPointTag::ankle_right]->getPoint());

//        temp.push_back(skeleton[KeyPointTag::shoulder_center]->getPoint()[0]); //0
//        temp.push_back(skeleton[KeyPointTag::shoulder_center]->getPoint()[1]); //1
//        temp.push_back(0.0); //skeleton[KeyPointTag::shoulder_center]->getPoint()[2]; //2

//        temp.push_back(skeleton[KeyPointTag::head]->getPoint()[0]); //3
//        temp.push_back(skeleton[KeyPointTag::head]->getPoint()[1]); //4
//        temp.push_back(0.0); //skeleton[KeyPointTag::head]->getPoint()[2]; //5

//        temp.push_back(skeleton[KeyPointTag::shoulder_left]->getPoint()[0]); //6
//        temp.push_back(skeleton[KeyPointTag::shoulder_left]->getPoint()[1]); //7
//        temp.push_back(0.0); //skeleton[KeyPointTag::shoulder_left]->getPoint()[2]; //8

//        temp.push_back(skeleton[KeyPointTag::elbow_left]->getPoint()[0]); //9
//        temp.push_back(skeleton[KeyPointTag::elbow_left]->getPoint()[1]); //10
//        temp.push_back(0.0); //skeleton[KeyPointTag::elbow_left]->getPoint()[2]; //11

//        temp.push_back(skeleton[KeyPointTag::hand_left]->getPoint()[0]); //12
//        temp.push_back(skeleton[KeyPointTag::hand_left]->getPoint()[1]); //13
//        temp.push_back(0.0); //skeleton[KeyPointTag::hand_left]->getPoint()[2]; //14

//        temp.push_back(skeleton[KeyPointTag::shoulder_right]->getPoint()[0]); //15
//        temp.push_back(skeleton[KeyPointTag::shoulder_right]->getPoint()[1]); //16
//        temp.push_back(0.0); //skeleton[KeyPointTag::shoulder_right]->getPoint()[2]; /17

//        temp.push_back(skeleton[KeyPointTag::elbow_right]->getPoint()[0]); //18
//        temp.push_back(skeleton[KeyPointTag::elbow_right]->getPoint()[1]); //19
//        temp.push_back(0.0); //skeleton[KeyPointTag::elbow_right]->getPoint()[2]; //20

//        temp.push_back(skeleton[KeyPointTag::hand_right]->getPoint()[0]); //21
//        temp.push_back(skeleton[KeyPointTag::hand_right]->getPoint()[1]); //22
//        temp.push_back(0.0); //skeleton[KeyPointTag::hand_right]->getPoint()[2]; //23

//        temp.push_back(skeleton[KeyPointTag::hip_center]->getPoint()[0]); //24
//        temp.push_back(skeleton[KeyPointTag::hip_center]->getPoint()[1]); //25
//        temp.push_back(0.0); //skeleton[KeyPointTag::hip_center]->getPoint()[2]; //26

//        temp.push_back(skeleton[KeyPointTag::hip_left]->getPoint()[0]); //27
//        temp.push_back(skeleton[KeyPointTag::hip_left]->getPoint()[1]); //28
//        temp.push_back(0.0); //skeleton[KeyPointTag::hip_left]->getPoint()[2]; //29

//        temp.push_back(skeleton[KeyPointTag::knee_left]->getPoint()[0]); //30
//        temp.push_back(skeleton[KeyPointTag::knee_left]->getPoint()[1]);//31
//        temp.push_back(0.0); //skeleton[KeyPointTag::knee_left]->getPoint()[2]; //32

//        temp.push_back(skeleton[KeyPointTag::ankle_left]->getPoint()[0]); //33
//        temp.push_back(skeleton[KeyPointTag::ankle_left]->getPoint()[1]); //34
//        temp.push_back(0.0); //skeleton[KeyPointTag::ankle_left]->getPoint()[2]; //35

//        temp.push_back(skeleton[KeyPointTag::hip_right]->getPoint()[0]); //36
//        temp.push_back(skeleton[KeyPointTag::hip_right]->getPoint()[1]); //37
//        temp.push_back(0.0); //skeleton[KeyPointTag::hip_right]->getPoint()[2]; //38

//        temp.push_back(skeleton[KeyPointTag::knee_right]->getPoint()[0]); //39
//        temp.push_back(skeleton[KeyPointTag::knee_right]->getPoint()[1]); //40
//        temp.push_back(0.0); //skeleton[KeyPointTag::knee_right]->getPoint()[2]; //41

//        temp.push_back(skeleton[KeyPointTag::ankle_right]->getPoint()[0]); //42
//        temp.push_back(skeleton[KeyPointTag::ankle_right]->getPoint()[1]); //43
//        temp.push_back(0.0); //skeleton[KeyPointTag::ankle_right]->getPoint()[2]; //44
    }

    void updateVec()
    {
        vector<Vector> temp1;
        temp1.clear();
        createVec(skeletonTemplate,temp1);
        skeleton_template.push_back(temp1);

        vector<Vector> temp2;
        temp2.clear();
        createVec(skeletonIn,temp2);
        skeleton_candidate.push_back(temp2);
    }

    bool updateModule() override
    {
        //if we query the database
        if(opcPort.getOutputCount() > 0 && start)
        {
            //get skeleton
            getSkeleton();

            //if the skeleton has been updated
            if(updated)
            {
                //update vectors to align
                updateVec();

                if( (Time::now()-tstart)> T )
                {
                    Dtw *dtw;
                    dtw=new Dtw(win,skeleton_template.size(),skeleton_candidate.size());

                    //for each keypoint
                    for(int i=0;i<skeletonIn.getNumKeyPoints();i++)
                    {
                        //for each component (xyz)
                        for(int l=0; l<3; l++)
                        {
                            //for each sample over time
                            for(int j=0;j<skeleton_template.size();j++)
                            {
                                s_template.push_back(skeleton_template[j][i][l]);
                                s_candidate.push_back(skeleton_candidate[j][i][l]);
                            }

                            double d=dtw->computeDistance(s_template,s_candidate);

                            if(d>dtw_thresh)
                            {
                                //check differences in speed
                                int ft = performFFT(s_template,"template",i);
                                int fc = performFFT(s_candidate,"test",i);
                                outfile << "\n";

                                if(ft-fc > 0)
                                    yInfo() << "move" << skeletonIn[i]->getTag() << "faster!" << ft << fc << d;
                                else if(ft-fc < 0)
                                    yInfo() << "move" << skeletonIn[i]->getTag() << "slower!" << ft << fc << d;

                                //check differences in position
                                s_aligned.resize(s_template.size());
                                s_aligned=dtw->align();
                                double errpos = 0.0;
                                for(int k=0; k<s_aligned.size(); k++)
                                {
                                    errpos += s_aligned[k]-s_template[k];
                                    if(k%10 == 0)
                                    {
                                        errpos /= 10.0;
//                                        if(abs(errpos) > errpos_thresh)
//                                        {
//                                            if(l%3 == 0)
//                                            {
//                                                if(errpos > 0.0)
//                                                    yInfo() << "move" << skeletonIn[i]->getTag() << "left!" << errpos;
//                                                else
//                                                    yInfo() << "move" << skeletonIn[i]->getTag() << "right!" << errpos;
//                                            }
//                                            else if(l%3 == 1)
//                                            {
//                                                if(errpos > 0.0)
//                                                    yInfo() << "move" << skeletonIn[i]->getTag() << "up!" << errpos;
//                                                else
//                                                    yInfo() << "move" << skeletonIn[i]->getTag() << "down!" << errpos;
//                                            }
////                                            else if(l%3 == 2)
////                                            {
////                                                if(errpos > 0.0)
////                                                    yInfo() << "move" << skeletonIn[i]->getTag() << "forward!" << errpos;
////                                                else
////                                                    yInfo() << "move" << skeletonIn[i]->getTag() << "backward!" << errpos;
////                                            }
//                                        }

                                        errpos = 0.0;
                                    }
                                }
                            }

                            s_template.clear();
                            s_candidate.clear();
                            s_aligned.clear();
                        }


                    }
                    cout << endl;

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
            }
        }
        return true;
    }

    vector<pair<double,int>> findPeaks(const vector<double> p)
    {
        vector<pair<double,int>> val;
        double noise = 5.0;
        //consider half of the spectrum
        for(int i=0;i<p.size()/2;i++)
        {
            if((i-1)>0)
            {
                if(p[i]>p[i-1] && p[i]>p[i+1] && p[i]>noise)
                    val.push_back(make_pair(p[i],i));
            }
            else
                if(p[i]>p[i+1] && p[i]>noise)
                    val.push_back(make_pair(p[i],i));
        }
        sort(val.rbegin(),val.rend());

//        for(int j=0; j<val.size(); j++)
//            yInfo() << val[j].first << val[j].second;
//        cout << endl;

        return val;
    }

    int performFFT(const vector<double> &s, const string &name, const int &i)
    {
        int n = s.size();
        fftw_complex *in,*out;
        fftw_plan p;

        in = (fftw_complex*) fftw_malloc(n*sizeof(fftw_complex));
        out = (fftw_complex*) fftw_malloc(n*sizeof(fftw_complex));

        for(int j=0; j<n; j++)
        {
            in[j][0]=s[j];
            in[j][1]=0.0;
        }

        //create plan, which is the object containing the parameters required
        p = fftw_plan_dft_1d(n,in,out,FFTW_FORWARD,FFTW_ESTIMATE);

        //perform FFT
        fftw_execute(p);

        //compute power spectrum and the frequency content
        vector<double> psd;
        psd.clear();
        for(int j=0; j<n; j++)
            psd.push_back(pow(abs(out[j][0]),2.0) / (1.0/period)*n);
        vector<pair<double,int>> m = findPeaks(psd);

        outfile << "joint" << " " << i << " ";
        for(int j=0; j<n; j++)
            outfile << in[j][0] << " ";
        outfile << "\n";

        outfile << name << " ";
        for(int j=0; j<n; j++)
            outfile << psd[j] << " ";
        outfile << "\n";

//        yInfo() << name << i << " ";
//        for(int j=0; j<m.size(); j++)
//            yInfo() << m[j];

        fftw_destroy_plan(p);
        fftw_free(in);
        fftw_free(out);

        if(m[0].second==0 && m.size()>1)
            return m[1].second;
        else
            return m[0].second;
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
