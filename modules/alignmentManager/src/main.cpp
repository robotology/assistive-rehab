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
#include "AssistiveRehab/dtw.h"
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
    int win,range_freq;
    double T,tstart,dtw_thresh,period,var_thresh,skew_thresh,psd_noise;

    SkeletonWaist skeletonIn,skeletonTemplate;
    string skel_tag,template_tag;
    vector<vector<Vector>> skeleton_template,skeleton_candidate;
    bool updated,start;
    vector<double> s_template,s_candidate,warped_template,warped_candidate;
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

    bool configure(ResourceFinder &rf) override
    {
        win = rf.check("win",Value(-1)).asDouble();
//        T = rf.check("T",Value(1.0)).asDouble();
        period = rf.check("period",Value(0.01)).asDouble();
        dtw_thresh = rf.check("dtw_thresh",Value(0.10)).asDouble();
        var_thresh = rf.check("var_thresh",Value(0.3)).asDouble();
        skew_thresh = rf.check("skew_thresh",Value(0.8)).asDouble();
        psd_noise = rf.check("psd_noise",Value(5.0)).asDouble();
        range_freq = rf.check("range_freq",Value(2)).asInt();

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
                    dtw=new Dtw(win);

                    //for each keypoint
                    for(int i=0;i<skeletonIn.getNumKeyPoints();i++)
                    {
                        //for each component (xyz)
                        int tot=0;
                        double ftavg=0.0, fcavg=0.0,davg=0.0;
                        for(int l=0;l<3;l++)
                        {
                            //for each sample over time
                            for(int j=0;j<skeleton_template.size();j++)
                            {
                                s_template.push_back(skeleton_template[j][i][l]);
                                s_candidate.push_back(skeleton_candidate[j][i][l]);
                            }
                            dtw->align(s_template,s_candidate,warped_template,warped_candidate);

                            outfile << i << " ";
                            for(int k=0; k<s_template.size(); k++)
                                outfile << s_template[k] << " ";
                            outfile << "\n";

                            outfile << i << " ";
                            for(int k=0; k<s_candidate.size(); k++)
                                outfile << s_candidate[k] << " ";
                            outfile << "\n";

                            outfile << i << " " << warped_template.size();
                            for(int k=0; k<warped_template.size(); k++)
                                outfile << warped_template[k] << " ";
                            outfile << "\n";

                            outfile << i << " " << warped_candidate.size();
                            for(int k=0; k<warped_candidate.size(); k++)
                                outfile << warped_candidate[k] << " ";
                            outfile << "\n";

                            double d=dtw->getDistance();
                            davg+=d;

                            int ft = performFFT(s_template,"template",i);
                            int fc = performFFT(s_candidate,"test",i);
                            if(ft!=-1)
                            {
                                ftavg += ft;
                                tot++;
                            }
                            if(fc!=-1)
                            {
                                fcavg += fc;
                                tot++;
                            }
                            
                            if(d>dtw_thresh && ft!=-1 && fc!=-1)
                            {
                                //evaluate error distribution
                                double errpos[warped_template.size()];
                                for(int k=0; k<warped_template.size(); k++)
                                {
                                    errpos[k] = warped_candidate[k]-warped_template[k];
                                }
                                double var = gsl_stats_variance(errpos,1,warped_template.size());
                                double skwns = gsl_stats_skew(errpos,1,warped_template.size());

                                if(var<var_thresh && fabs(skwns)<skew_thresh)
                                    yWarning() << "moving well" << var << skwns;
                                else if(var<var_thresh && skwns>0.0) //there's a tail on the right of the error distribution
                                {
                                    if(l%3==0) //x component
                                        yWarning() << "move" << skeletonIn[i]->getTag() << "left!" << var << skwns;
                                    else if(l%3==1) //y component
                                        yWarning() << "move" << skeletonIn[i]->getTag() << "up!" << var << skwns;
                                    else if(l%3==2) //z component
                                        yWarning() << "move" << skeletonIn[i]->getTag() << "forward!" << var << skwns;
                                }
                                else if(var<var_thresh && skwns<0.0) //there's a tail on the left of the error distribution
                                {
                                    if(l%3==0) //x component
                                        yWarning() << "move" << skeletonIn[i]->getTag() << "right!" << var << skwns;
                                    else if(l%3==1) //y component
                                        yWarning() << "move" << skeletonIn[i]->getTag() << "down!" << var << skwns;
                                    else if(l%3==2) //z component
                                        yWarning() << "move" << skeletonIn[i]->getTag() << "backward!" << var << skwns;
                                }
                                else if(var>var_thresh)
                                    yWarning() << "move" << skeletonIn[i]->getTag() << "better!" << var << skwns;
                            }

                            s_template.clear();
                            s_candidate.clear();
                        }

                        //check differences in speed
                        ftavg/=tot;
                        fcavg/=tot;
                        davg/=3.0;
                        if(davg > dtw_thresh)
                        {
                            //-1 if joint is stale
                            if(ftavg!=-1 && fcavg!=-1)
                            {
                                if(ftavg-fcavg != 0)
                                {
                                    if(fcavg == 0)
                                        yWarning() << "move" << skeletonIn[i]->getTag() << ftavg << fcavg << davg;
                                    else if(ftavg == 0)
                                        yWarning() << "stop" << skeletonIn[i]->getTag() << ftavg << fcavg << davg;
                                    else if(ftavg-fcavg > range_freq)
                                        yWarning() << "move" << skeletonIn[i]->getTag() << "faster!" << ftavg << fcavg << davg;
                                    else if(ftavg-fcavg < -range_freq)
                                        yWarning() << "move" << skeletonIn[i]->getTag() << "slower!" << ftavg << fcavg << davg;
                                }
                            }
                        }
                    }
                    cout << endl;

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
        //consider half of the spectrum
        for(int i=0;i<p.size()/2;i++)
        {
            if((i-1)>0)
            {
                if(p[i]>p[i-1] && p[i]>p[i+1] && p[i]>psd_noise)
                    val.push_back(make_pair(p[i],i));
            }
            else
                if(p[i]>p[i+1] && p[i]>psd_noise)
                    val.push_back(make_pair(p[i],i));
        }
        //sort psd by highest value
        sort(val.rbegin(),val.rend());

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

//        outfile << "joint" << " " << i << " ";
//        outfile << i << " ";
//        for(int j=0; j<n; j++)
//            outfile << in[j][0] << " ";
//        outfile << "\n";

//        outfile << name << " ";
//        for(int j=0; j<n; j++)
//            outfile << psd[j] << " ";
//        outfile << "\n";

//        yInfo() << name << i << " ";
//        for(int j=0; j<m.size(); j++)
//            yInfo() << m[j].second;

        fftw_destroy_plan(p);
        fftw_free(in);
        fftw_free(out);

        if(m.size()>0)
        {
            if(m.size()>1 && m[0].second==0)
                return m[1].second;
            else
                return m[0].second;
        }
        else
        {
  //          yWarning() << "PSD empty.." << skeletonIn[i]->getTag() << "stale?";
            return -1;
        }

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
