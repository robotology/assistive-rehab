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
#include "src/feedbackProducer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace assistive_rehab;

/********************************************************/
class Feedback : public RFModule, public feedbackProducer_IDL
{
private:
    //ports
    RpcServer rpcPort;
    RpcClient opcPort;
    RpcClient analyzerPort;
    BufferedPort<Bottle> actionPort;
    BufferedPort<Bottle> outPort;

    //parameters
    int win,filter_order;
    double period;

    Mutex mutex;

    SkeletonWaist skeletonIn,skeletonTemplate;
    string skel_tag,template_tag,metric_tag;
    vector<vector<Vector>> skeleton_template,skeleton_candidate;
    bool updated,started,first_run;
    vector<string> joint_list;
    Matrix feedback_thresholds;
    Vector ref;

public:

    /****************************************************************/
    bool attach(RpcServer &source) override
    {
        return yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool setFeedbackThresh(const Matrix &feedback_thresholds_) override
    {
        LockGuard lg(mutex);
        feedback_thresholds = feedback_thresholds_;
        yInfo() << "feedback thresholds" << feedback_thresholds.toString();
        return true;
    }

    /****************************************************************/
    bool setTemplateTag(const string &template_tag_) override
    {
        LockGuard lg(mutex);
        template_tag = template_tag_;
        yInfo() << "Template skeleton" << template_tag;
        return true;
    }

    /****************************************************************/
    bool setSkelTag(const string &skel_tag_) override
    {
        LockGuard lg(mutex);
        skel_tag = skel_tag_;
        yInfo() << "Tag skeleton" << skel_tag;
        return true;
    }

    /****************************************************************/
    bool setMetric(const string &metric_tag_) override
    {
        LockGuard lg(mutex);
        metric_tag = metric_tag_;
        yInfo() << "Metric tag" << metric_tag;
        return true;
    }

    /****************************************************************/
    bool start() override
    {
        LockGuard lg(mutex);
        started = true;
        first_run = true;
        return true;
    }

    /****************************************************************/
    bool stop() override
    {
        LockGuard lg(mutex);
        skel_tag.clear();
        template_tag.clear();
        joint_list.clear();
        skeleton_template.clear();
        skeleton_candidate.clear();
        started = false;
        first_run = true;
        return true;
    }

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
    bool configure(ResourceFinder &rf) override
    {
        win = rf.check("win",Value(-1)).asDouble();
        period = rf.check("period",Value(0.01)).asDouble();
        filter_order = rf.check("filter_order",Value(3)).asInt();

        opcPort.open("/feedbackProducer/opc");
        outPort.open("/feedbackProducer:o");
        analyzerPort.open("/feedbackProducer/analyzer:rpc");
        actionPort.open("/feedbackProducer/action:i");
        rpcPort.open("/feedbackProducer/rpc");
        attach(rpcPort);

        started = false;

        return true;
    }

    /********************************************************/
    bool interruptModule() override
    {
        opcPort.interrupt();
        outPort.interrupt();
        analyzerPort.interrupt();
        actionPort.interrupt();
        rpcPort.interrupt();
        yInfo() << "Interrupted module";
        return true;
    }

    /********************************************************/
    bool close() override
    {
        opcPort.close();
        outPort.close();
        analyzerPort.close();
        actionPort.close();
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
    void fill(const SkeletonWaist& skeleton, vector<Vector>& temp)
    {
        for(size_t i=0; i<joint_list.size(); i++)
        {
            temp.push_back(skeleton[joint_list[i]]->getPoint());
        }
    }

    /********************************************************/
    void updateVec()
    {
        vector<Vector> temp1;
        temp1.clear();
        fill(skeletonTemplate,temp1);
        skeleton_template.push_back(temp1);

        vector<Vector> temp2;
        temp2.clear();
        fill(skeletonIn,temp2);
        skeleton_candidate.push_back(temp2);
    }

    /********************************************************/
    void produceFeedback(const int idx_joint, const vector<int> &freqt, const vector<int> &freqc,
                         const vector<double> &stats, Bottle &feedback)
    {
        double sx_thresh = feedback_thresholds[0][idx_joint];
        double sy_thresh = feedback_thresholds[1][idx_joint];
        double sz_thresh = feedback_thresholds[2][idx_joint];
        double range_freq = feedback_thresholds[3][idx_joint];

        double devx = stats[0];
        double skwnx = stats[1];
        int ftx = freqt[0];
        int fcx = freqc[0];

        double devy = stats[2];
        double skwny = stats[3];
        int fty = freqt[1];
        int fcy = freqc[1];

        double devz = stats[4];
        double skwnz = stats[5];
        int ftz = freqt[2];
        int fcz = freqc[2];

        bool joint_templ_stale = ftx < 0 && fty < 0 && ftz < 0;
        bool joint_skel_stale = fcx < 0 && fcy < 0 && fcz < 0;

        if(!joint_templ_stale && !joint_skel_stale)
        {
            /*********************/
            /*    FIRST CHECK    */
            /*   error in speed  */
            /*********************/
            int dfx = fcx-ftx;
            int dfy = fcy-fty;
            int dfz = fcz-ftz;
            bool fxy_pos = dfx > range_freq && dfy > range_freq;
            bool fxz_pos = dfx > range_freq && dfz > range_freq;
            bool fyz_pos = dfy > range_freq && dfz > range_freq;
            bool fxy_neg = dfx < -range_freq && dfy < -range_freq;
            bool fxz_neg = dfx < -range_freq && dfz < -range_freq;
            bool fyz_neg = dfy < -range_freq && dfz < -range_freq;

            if(fxy_pos || fxz_pos || fyz_pos)
            {
                feedback.addString("speed");
                feedback.addString("pos");
                return;
            }
            else if(fxy_neg || fxz_neg || fyz_neg)
            {
                feedback.addString("speed");
                feedback.addString("neg");
                return;
            }

            /************************/
            /*     SECOND CHECK     */
            /*   error in position  */
            /************************/
            //we check the error in position
            bool errx = devx > sx_thresh;
            bool erry = devy > sy_thresh;
            bool errz = devz > sz_thresh;
            if( errx || erry || errz )
            {
                if(errx)
                {
                    feedback.addString("position-rom");
                    if(skwnx > 0.0)
                    {
                        feedback.addString("pos x");
                        return;
                    }
                    else
                    {
                        feedback.addString("neg x");
                        return;
                    }
                }
                if(erry)
                {
                    feedback.addString("position-rom");
                    if(skwny > 0.0)
                    {
                        feedback.addString("pos y");
                        return;
                    }
                    else
                    {
                        feedback.addString("neg y");
                        return;
                    }
                }
                if(errz)
                {
                    feedback.addString("position-rom");
                    if(skwnz > 0.0)
                    {
                        feedback.addString("pos z");
                        return;
                    }
                    else
                    {
                        feedback.addString("neg z");
                        return;
                    }
                }
            }
            feedback.addString("perfect");
        }
    }

    /********************************************************/
    void produceFeedback(const int idx_joint, const double &maxt, const double &maxc, Bottle &feedback)
    {
        double target_thresh = feedback_thresholds[0][idx_joint];
        if( maxc-maxt > target_thresh )
        {
            feedback.addString("position-ep");
            feedback.addString("pos");
            return;
        }
        else if( maxc-maxt < -target_thresh )
        {
            feedback.addString("position-ep");
            feedback.addString("neg");
            return;
        }
        feedback.addString("perfect");
    }

    /********************************************************/
    void analyzeRom(Bottle &outfeedback)
    {
        Bottle &f=outfeedback.addList();
        vector<double> joint_template,joint_candidate,warped_template,warped_candidate;

        Dtw *dtw;
        dtw=new Dtw(win);

        //for each keypoint in the list
        for(int i=0;i<joint_list.size();i++)
        {
            Bottle &feedbackjoint=f.addList();
            string tag=joint_list[i];
            vector<int> freqt,freqc;
            vector<double> stats;

            //for each component (xyz)
            for(int l=0;l<3;l++)
            {
                //for each sample over time
                for(int j=0;j<skeleton_template.size();j++)
                {
                    joint_template.push_back(skeleton_template[j][i][l]);
                    joint_candidate.push_back(skeleton_candidate[j][i][l]);
                }

                /****************/
                /*     DTW      */
                /****************/
                dtw->align(joint_template,joint_candidate,warped_template,warped_candidate);

                /********************************/
                /*     Difference in speed      */
                /********************************/
                double maxpsdt,maxpsdc;
                int ft = performFFT(joint_template,maxpsdt);
                int fc = performFFT(joint_candidate,maxpsdc);
                freqt.push_back(ft);
                freqc.push_back(fc);

                /********************************/
                /*    Difference in position    */
                /********************************/
                double errpos[warped_template.size()];
                for(int k=0; k<warped_template.size(); k++)
                {
                    errpos[k] = warped_candidate[k]-warped_template[k];
                }
                double sdev = gsl_stats_sd(errpos,1,warped_template.size());
                double skwns = gsl_stats_skew(errpos,1,warped_template.size());
                stats.push_back(sdev);
                stats.push_back(skwns);

                joint_template.clear();
                joint_candidate.clear();
                warped_template.clear();
                warped_candidate.clear();
            }

            //produce feedback for a single joint
            feedbackjoint.addString(tag);
            produceFeedback(i,freqt,freqc,stats,feedbackjoint);
        }

        delete dtw;
    }

    /********************************************************/
    void analyzeEP(Bottle &outfeedback)
    {
        vector<double> joint_template,joint_candidate;
        Bottle &f=outfeedback.addList();

        Vector xth,yth,zth,xch,ych,zch;
        for(int i=0;i<joint_list.size();i++)
        {
            Bottle &feedbackjoint=f.addList();
            string tag=joint_list[i];

            //for each component (xyz)
            for(int l=0;l<3;l++)
            {
                //for each sample over time
                for(int j=0;j<skeleton_template.size();j++)
                {
                    joint_template.push_back(skeleton_template[j][i][l]);
                    joint_candidate.push_back(skeleton_candidate[j][i][l]);
                }

                /*******************************************/
                /*    Difference in reaching the target    */
                /*******************************************/
                if(l%3==0)
                {
                    for(int k=0; k<joint_template.size(); k++)
                    {
                        xth.push_back(joint_template[k]-ref[0]);
                        xch.push_back(joint_candidate[k]-ref[0]);
                    }
                }
                if(l%3==1)
                {
                    for(int k=0; k<joint_template.size(); k++)
                    {
                        yth.push_back(joint_template[k]-ref[1]);
                        ych.push_back(joint_candidate[k]-ref[1]);
                    }
                }
                if(l%3==2)
                {
                    for(int k=0; k<joint_template.size(); k++)
                    {
                        zth.push_back(joint_template[k]-ref[2]);
                        zch.push_back(joint_candidate[k]-ref[2]);
                    }
                }

                joint_template.clear();
                joint_candidate.clear();
            }

            Vector len_template_hand,len_candidate_hand;
            for(int k=0; k<xth.size(); k++)
            {
                double lt = sqrt(pow(xth[k],2)+pow(yth[k],2)+pow(zth[k],2));
                double lc = sqrt(pow(xch[k],2)+pow(ych[k],2)+pow(zch[k],2));
                len_template_hand.push_back(lt);
                len_candidate_hand.push_back(lc);
            }
            double maxt = yarp::math::findMax(len_template_hand);
            double maxc = yarp::math::findMax(len_candidate_hand);
            yInfo() << "Max displacement between targets" << maxc-maxt << maxt << maxc;

            //produce feedback for a single joint
            feedbackjoint.addString(tag);
            produceFeedback(i,maxt,maxc,feedbackjoint);
        }
    }

    /********************************************************/
    bool updateModule() override
    {
        LockGuard lg(mutex);

        //if we query the database
        if(opcPort.getOutputCount() > 0 && started)
        {
            if(first_run)
            {
                Bottle cmd,rep;
                cmd.addString("listJoints");
                if(analyzerPort.write(cmd,rep))
                {
                    Bottle &bJoint=*rep.get(0).asList();
                    if(bJoint.size()>0)
                    {
                        for(size_t i=0; i<bJoint.size(); i++)
                            joint_list.push_back(bJoint.get(i).asString());
                    }
                }
                yInfo()<<"Joints under analysis:"<<joint_list;

                if(metric_tag.find("EP") != std::string::npos)
                {
                    ref=skeletonIn[joint_list[0]]->getParent(0)->getParent(0)->getPoint();
                }

                first_run = false;
            }

            //get skeleton
            getSkeleton();

            //if the skeleton has been updated
            if(updated)
            {
                //update vectors to align
                updateVec();

                if(Bottle *target=actionPort.read(false))
                {
                    yInfo() << skeleton_template.size() << "skeleton samples";
                    yInfo() << skeleton_template[0].size() << "joints in the skeleton";
                    yInfo() << skeleton_template[0][0].size() << "components for each joint in the skeleton";

                    Bottle &outfeedback=outPort.prepare();
                    outfeedback.clear();

                    string exercise = target->get(0).asString();
                    string action = target->get(1).asString();
                    double confidence = target->get(2).asDouble();
                    yInfo() << exercise << action << confidence;
                    if(action == exercise && confidence > 0.0)
                    {
                        //analysis for ROM
                        if(metric_tag.find("ROM") != std::string::npos)
                        {
                            yInfo() << "Analyzing ROM";
                            analyzeRom(outfeedback);
                        }

                        //analysis for EP
                        if(metric_tag.find("EP") != std::string::npos)
                        {
                            yInfo() << "Analyzing EP";
                            analyzeEP(outfeedback);
                        }
                    }
                    else
                    {
                        Bottle &bList=outfeedback.addList();
                        Bottle &wrong=bList.addList();
                        wrong.addString("wrong");
                    }

                    yInfo() << "Sending feedback";
                    outPort.write();

                    skeleton_template.clear();
                    skeleton_candidate.clear();
                }
            }
        }

        return true;
    }

    /********************************************************/
    int findMax(const vector<Vector> &p, double &max)
    {
        int imax = 1;
        max = p[imax][0];
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
        else
            return imax;
    }

    /********************************************************/
    int performFFT(const vector<double> &s, double &max)
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
        int f = findMax(psd,max);

        fftw_destroy_plan(p);
        fftw_free(in);
        fftw_free(out);

        return f;
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

    Feedback feedback;

    return feedback.runModule(rf);
}
