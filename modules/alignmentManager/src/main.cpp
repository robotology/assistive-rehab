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
#include <algorithm>
#include <fstream>
#include <fftw3.h>
#include <gsl/gsl_statistics.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/filters.h>
#include "AssistiveRehab/dtw.h"
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace assistive_rehab;

/********************************************************/
class Aligner : public RFModule
{
private:
    //ports
    RpcServer rpcPort;
    RpcClient opcPort;
    BufferedPort<Bottle> outPort;

    //parameters
    int win,filter_order;
    double T,tstart,dtw_thresh,period,psd_noise;

    SkeletonWaist skeletonIn,skeletonTemplate;
    string skel_tag,template_tag;
    vector<vector<Vector>> skeleton_template,skeleton_candidate;
    bool updated,start;
    vector<double> joint_template,joint_candidate,warped_template,warped_candidate;
    ofstream outfile;


public:

    /********************************************************/
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
                                            skeletonIn.normalize(); //we normalize to avoid differences due to different physiques
                                            updated = true;
                                            delete skeleton;
                                        }
                                        if(prop.check("tag") && tag==template_tag)
                                        {
                                            Skeleton* skeleton = skeleton_factory(prop);
                                            skeletonTemplate.update(skeleton->toProperty());
                                            skeletonTemplate.normalize();
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

    /********************************************************/
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
            yInfo() << "Check every" << T << "seconds";

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

    /********************************************************/
    bool configure(ResourceFinder &rf) override
    {
        win = rf.check("win",Value(-1)).asDouble();
        period = rf.check("period",Value(0.01)).asDouble();
        dtw_thresh = rf.check("dtw_thresh",Value(0.10)).asDouble();
        psd_noise = rf.check("psd_noise",Value(1000.0)).asDouble();
        filter_order = rf.check("filter_order",Value(3)).asInt();

        opcPort.open("/alignmentManager/opc");
        outPort.open("/alignmentManager:o");
        rpcPort.open("/alignmentManager/rpc");
        attach(rpcPort);

        outfile.open("dtw-test.txt");

        start = false;

        return true;
    }

    /********************************************************/
    bool interruptModule() override
    {
        opcPort.interrupt();
        outPort.interrupt();
        rpcPort.interrupt();
        yInfo() << "Interrupted module";
        return true;
    }

    /********************************************************/
    bool close() override
    {
        outfile.close();

        opcPort.close();
        outPort.close();
        rpcPort.close();
        yInfo() << "Closed ports";
        return true;
    }

    /********************************************************/
    double getPeriod() override
    {
        return period;
    }

    /********************************************************/
    void fillOrdered(const SkeletonWaist& skeleton, vector<Vector>& temp)
    {
        //same order as in skeletonWaist
        temp.push_back(skeleton[KeyPointTag::shoulder_center]->getPoint()); //0
        temp.push_back(skeleton[KeyPointTag::head]->getPoint()); //1
        temp.push_back(skeleton[KeyPointTag::shoulder_left]->getPoint()); //2
        temp.push_back(skeleton[KeyPointTag::elbow_left]->getPoint()); //3
        temp.push_back(skeleton[KeyPointTag::hand_left]->getPoint()); //4
        temp.push_back(skeleton[KeyPointTag::shoulder_right]->getPoint()); //5
        temp.push_back(skeleton[KeyPointTag::elbow_right]->getPoint()); //6
        temp.push_back(skeleton[KeyPointTag::hand_right]->getPoint()); //7
        temp.push_back(skeleton[KeyPointTag::hip_center]->getPoint()); //8
        temp.push_back(skeleton[KeyPointTag::hip_left]->getPoint()); //9
        temp.push_back(skeleton[KeyPointTag::knee_left]->getPoint()); //10
        temp.push_back(skeleton[KeyPointTag::ankle_left]->getPoint()); //11
        temp.push_back(skeleton[KeyPointTag::hip_right]->getPoint());
        temp.push_back(skeleton[KeyPointTag::knee_right]->getPoint());
        temp.push_back(skeleton[KeyPointTag::ankle_right]->getPoint());
    }

    /********************************************************/
    void updateVec()
    {
        vector<Vector> temp1;
        temp1.clear();
        fillOrdered(skeletonTemplate,temp1);
        skeleton_template.push_back(temp1);

        vector<Vector> temp2;
        temp2.clear();
        fillOrdered(skeletonIn,temp2);
        skeleton_candidate.push_back(temp2);
    }

    /********************************************************/
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
                    Bottle &outList=outPort.prepare();
                    outList.clear();

                    Dtw *dtw;
                    dtw=new Dtw(win);

                    //for each keypoint
                    Bottle &out=outList.addList();
                    for(int i=0;i<skeletonIn.getNumKeyPoints();i++)
                    {
                        Bottle &feedback=out.addList();
                        feedback.addString(skeletonIn[i]->getTag());

                        //for each component (xyz)
                        for(int l=0;l<3;l++)
                        {
                            //for each sample over time
                            for(int j=0;j<skeleton_template.size();j++)
                            {
                                joint_template.push_back(skeleton_template[j][i][l]);
                                joint_candidate.push_back(skeleton_candidate[j][i][l]);
                            }
                            dtw->align(joint_template,joint_candidate,warped_template,warped_candidate);
                            double d=dtw->getDistance();

//                            outfile << i << " ";
//                            for(int k=0; k<joint_template.size(); k++)
//                                outfile << joint_template[k] << " ";
//                            outfile << "\n";

//                            outfile << i << " ";
//                            for(int k=0; k<joint_candidate.size(); k++)
//                                outfile << joint_candidate[k] << " ";
//                            outfile << "\n";

//                            outfile << i << " " << warped_template.size();
//                            for(int k=0; k<warped_template.size(); k++)
//                                outfile << warped_template[k] << " ";
//                            outfile << "\n";

//                            outfile << i << " " << warped_candidate.size();
//                            for(int k=0; k<warped_candidate.size(); k++)
//                                outfile << warped_candidate[k] << " ";
//                            outfile << "\n";


                            int ft = performFFT(joint_template,"template",i);
                            int fc = performFFT(joint_candidate,"candidate",i);

                            /********************************/
                            /*    Difference in position    */
                            /********************************/
                            Bottle &fout=feedback.addList();
                            if(l%3==0) //x component
                                fout.addString("feed_pos_x");
                            else if(l%3==1) //y component
                                fout.addString("feed_pos_y");
                            else if(l%3==2) //z component
                                fout.addString("feed_pos_z");
                            fout.addDouble(d);
                            if(d>dtw_thresh && (ft!=-1 && fc!=-1))
                            {
                                //evaluate error distribution
                                double errpos[warped_template.size()];
                                for(int k=0; k<warped_template.size(); k++)
                                {
                                    errpos[k] = warped_candidate[k]-warped_template[k];
                                }
                                //the kurtosis for standard normal distribution is 3
                                double kurt = 3.0 + gsl_stats_kurtosis(errpos,1,warped_template.size());
                                double skwns = gsl_stats_skew(errpos,1,warped_template.size());

                                fout.addDouble(kurt);
                                fout.addDouble(skwns);

                                fout.addString("feed_speed");
                                fout.addInt(ft);
                                fout.addInt(fc);
                            }
                            else
                            {
                                fout.addDouble(0.0);
                                fout.addDouble(0.0);

                                fout.addString("feed_speed");
                                fout.addInt(0);
                                fout.addInt(0);
                            }

                            joint_template.clear();
                            joint_candidate.clear();
                        }
                    }

                    yInfo() << "Sending feedback";
                    outPort.write();

                    tstart=Time::now();
                    skeleton_template.clear();
                    skeleton_candidate.clear();

                    delete dtw;
                }
            }
        }
        return true;
    }

    /********************************************************/
    int findMax(const vector<Vector> &p)
    {
        int imax = 1;
        double max = p[imax][0];
        //consider half of the spectrum
        for(size_t i=2;i<p.size()/2;i++)
        {
            if(p[i][0] > max)
            {
                imax = i;
                max = p[i][0];
            }
        }

        if(max == 0)
            return -1; //joint is stale
        else if(max > psd_noise)
            return imax;
        else
            return 0;
    }

    /********************************************************/
    int performFFT(const vector<double> &s, const string &name, const int i)
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
        vector<Vector> psd;
        vector<Vector> filtered_psd;
        psd.clear();
        psd.resize(n);
        filtered_psd.clear();
        filtered_psd.resize(n);
        MedianFilter filter(filter_order,Vector(1,0.0));
        for(int j=0; j<n; j++)
        {
            psd[j].push_back(pow(abs(out[j][0]),2.0) / (1.0/period)*n);
            filtered_psd[j] = filter.filt(psd[j]);
        }
        int f = findMax(psd);

        outfile << "joint" << " " << i << " ";
        for(int j=0; j<n; j++)
            outfile << in[j][0] << " ";
        outfile << "\n";

        outfile << name << " ";
        for(int j=0; j<n; j++)
            outfile << psd[j][0] << " ";
        outfile << "\n";

        outfile << name << " ";
        for(int j=0; j<n; j++)
            outfile << filtered_psd[j][0] << " ";
        outfile << "\n";

        double errpos[warped_template.size()];
        outfile << name << " ";
        for(int k=0; k<warped_template.size(); k++)
        {
            errpos[k] = warped_candidate[k]-warped_template[k];
            outfile << errpos[k] << " ";
        }
        outfile << "\n";

        fftw_destroy_plan(p);
        fftw_free(in);
        fftw_free(out);

        return f;

//        if(m.size()>0)
//        {
//            if(m.size()>1 && m[0].second==0)
//                return m[1].second;
//            else
//                return m[0].second;
//        }
//        else
//        {
//            //          yWarning() << "PSD empty.." << skeletonIn[i]->getTag() << "stale?";
//            return -1;
//        }

    }
};

/********************************************************/
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
