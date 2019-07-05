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
#include <locale>
#include "AssistiveRehab/dtw.h"
#include "AssistiveRehab/skeleton.h"
#include "src/feedbackProducer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
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
    bool first;
    int use_robot_template,mirror_robot_template;

    Mutex mutex;

    SkeletonStd skeletonIn,skeletonTemplate;
    string skel_tag,template_tag,metric_tag;
    vector<vector<Vector>> skeleton_template,skeleton_candidate;
    bool updated,started;
    vector<string> joint_list;
    Matrix feedback_thresholds;
    double action_threshold;
    vector<double> target;
    Matrix T,T2;
    string part;

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
        yInfo() << "Feedback thresholds" << feedback_thresholds.toString();
        return true;
    }

    /****************************************************************/
    bool setTarget(const vector<double> &target_) override
    {
        LockGuard lg(mutex);
        target = target_;
        yInfo() << "Target to reach" << target;
        return true;
    }

    /****************************************************************/
    bool setTransformation(const Matrix &T_) override
    {
        LockGuard lg(mutex);
        T = T_;
        yInfo() << "Transformation matrix" << T.toString();
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
    bool setJoints(const vector<string> &joint_list_) override
    {
        LockGuard lg(mutex);
        yInfo() << "Joint list";
        joint_list.resize(joint_list_.size());
        for(size_t i=0; i<joint_list_.size(); i++)
        {
            joint_list[i] = joint_list_[i];
            yInfo() << joint_list[i];
        }
        return true;
    }

    /****************************************************************/
    bool setRobotTemplate(const int use_robot_template_,
                          const int mirror_robot_template_) override
    {
        LockGuard lg(mutex);
        use_robot_template = use_robot_template_;
        mirror_robot_template = mirror_robot_template_;
        yInfo() << "Using robot template";
        return true;
    }

    /****************************************************************/
    bool setPart(const string &part_) override
    {
        LockGuard lg(mutex);
        part=part_;
        yInfo() << "Analyzing"<<part<<"part";
        return true;
    }

    /****************************************************************/
    bool start() override
    {
        LockGuard lg(mutex);
        yInfo() << "Start!";
        started = true;
        first = true;
        return true;
    }

    /****************************************************************/
    bool stop() override
    {
        LockGuard lg(mutex);
        yInfo() << "Stop!";
        skel_tag.clear();
        template_tag.clear();
        joint_list.clear();
        target.clear();
        skeleton_template.clear();
        skeleton_candidate.clear();
        started = false;
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

    /********************************************************/
    bool configure(ResourceFinder &rf) override
    {
        win = rf.check("win",Value(-1)).asDouble();
        period = rf.check("period",Value(0.01)).asDouble();
        filter_order = rf.check("filter_order",Value(3)).asInt();
        action_threshold = rf.check("action-threshold",Value(0.3)).asDouble();

        opcPort.open("/feedbackProducer/opc");
        outPort.open("/feedbackProducer:o");
        analyzerPort.open("/feedbackProducer/analyzer:rpc");
        actionPort.open("/feedbackProducer/action:i");
        rpcPort.open("/feedbackProducer/rpc");
        attach(rpcPort);

        started = false;
        first = true;
        use_robot_template = 0;
        T = T2 = eye(4);

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
    void updateVec()
    {
        vector<Vector> temp1,temp2;
        temp1.clear();
        temp2.clear();
        for(size_t i=0; i<joint_list.size(); i++)
        {
            string tag_joint=joint_list[i];
            if(mirror_robot_template)
            {
                string mirrored_tag_joint;
                locale loc;
                string p=(toupper(part[0],loc))+part.substr(1,part.size());
                size_t found=tag_joint.find(p);
                if(part=="left")
                {
                    mirrored_tag_joint=tag_joint.substr(0,found)+"Right";
                }
                else
                {
                    mirrored_tag_joint=tag_joint.substr(0,found)+"Left";
                }
                temp1.push_back(skeletonTemplate[mirrored_tag_joint]->getPoint());
            }
            else
            {
                temp1.push_back(skeletonTemplate[tag_joint]->getPoint());
            }

            temp2.push_back(skeletonIn[tag_joint]->getPoint());
        }

        skeleton_template.push_back(temp1);
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

            yDebug() << "fx:" << fcx << ftx;
            yDebug() << "fy:" << fcy << fty;
            yDebug() << "fz:" << fcz << ftz;

            if(fxy_pos || fxz_pos || fyz_pos)
            {
                yDebug() << "fx:" << fcx << ftx;
                yDebug() << "fy:" << fcy << fty;
                yDebug() << "fz:" << fcz << ftz;
                feedback.addString("speed");
                feedback.addString("pos");
                return;
            }
            else if(fxy_neg || fxz_neg || fyz_neg)
            {
                yDebug() << "fx:" << fcx << ftx;
                yDebug() << "fy:" << fcy << fty;
                yDebug() << "fz:" << fcz << ftz;
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
            yDebug() << "devx:" << devx;
            yDebug() << "devy:" << devy;
            yDebug() << "devz:" << devz;
            if( errx || erry || errz )
            {
                if(errx)
                {
                    feedback.addString("position-rom");
                    if(skwnx > 0.0)
                    {
                        feedback.addString("neg z");
                        return;
                    }
                    else
                    {
                        feedback.addString("pos z");
                        return;
                    }
                }
                if(erry)
                {
                    feedback.addString("position-rom");
                    if(skwny > 0.0)
                    {
                        feedback.addString("neg x");
                        return;
                    }
                    else
                    {
                        feedback.addString("pos x");
                        return;
                    }
                }
                if(errz)
                {
                    feedback.addString("position-rom");
                    if(skwnz > 0.0)
                    {
                        feedback.addString("neg y");
                        return;
                    }
                    else
                    {
                        feedback.addString("pos y");
                        return;
                    }
                }
            }
            feedback.addString("perfect");
        }
    }

    /********************************************************/
    void produceFeedback(const int idx_joint, const Vector &dtemplate2target,
                         const Vector &dcandidate2target, Bottle &feedback)
    {
        int score_thresh = feedback_thresholds[1][idx_joint];
        double inliers_thresh = feedback_thresholds[2][idx_joint];

        //extract statistics from template
        double mu_template = gsl_stats_mean(dtemplate2target.data(),1,dtemplate2target.size());
        double std_template = gsl_stats_sd(dtemplate2target.data(),1,dtemplate2target.size());

        //count how many points are inside the template distribution
        int count=0;
        for(size_t i=0; i<dcandidate2target.size(); i++)
        {
            double zscorei=(dcandidate2target[i]-mu_template)/std_template;
            if(zscorei < score_thresh)
                count++;
        }

        //if the number of inliers is not enough, the target is not reached
        double inliers = ((double)count/dcandidate2target.size());
        yInfo()<< "Template statistics" << mu_template << std_template;
        yInfo()<< "Number of inliers:" << count << inliers;
        if(inliers < inliers_thresh)
        {
            feedback.addString("position-ep");
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

            Vector xth,yth,zth,xch,ych,zch;
            //for each component (xyz)
            for(int l=0;l<3;l++)
            {
                //for each sample over time
                for(int j=0;j<skeleton_template.size();j++)
                {
                    joint_template.push_back(skeleton_template[j][i][l]);
                    joint_candidate.push_back(skeleton_candidate[j][i][l]);
                }

                if(l%3==0)
                {
                    for(int k=0; k<joint_template.size(); k++)
                    {
                        xth.push_back(joint_template[k]);
                        xch.push_back(joint_candidate[k]);
                    }
                }
                if(l%3==1)
                {
                    for(int k=0; k<joint_template.size(); k++)
                    {
                        yth.push_back(joint_template[k]);
                        ych.push_back(joint_candidate[k]);
                    }
                }
                if(l%3==2)
                {
                    for(int k=0; k<joint_template.size(); k++)
                    {
                        zth.push_back(joint_template[k]);
                        zch.push_back(joint_candidate[k]);
                    }
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
            double radius = feedback_thresholds[0][i];

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
                        xth.push_back(joint_template[k]);
                        xch.push_back(joint_candidate[k]);
                    }
                }
                if(l%3==1)
                {
                    for(int k=0; k<joint_template.size(); k++)
                    {
                        yth.push_back(joint_template[k]);
                        ych.push_back(joint_candidate[k]);
                    }
                }
                if(l%3==2)
                {
                    for(int k=0; k<joint_template.size(); k++)
                    {
                        zth.push_back(joint_template[k]);
                        zch.push_back(joint_candidate[k]);
                    }
                }

                joint_template.clear();
                joint_candidate.clear();
            }

            Vector dist_template2target,dist_candidate2target;
            for(int k=0; k<xth.size(); k++)
            {
                double dtt = pow(xth[k]-target[0],2)+pow(yth[k]-target[1],2)+pow(zth[k]-target[2],2);
                double dct = pow(xch[k]-target[0],2)+pow(ych[k]-target[1],2)+pow(zch[k]-target[2],2);
                if(dtt < pow(radius,2))
                {
                    dist_template2target.push_back(dtt);
                }
                dist_candidate2target.push_back(dct);
            }

            //produce feedback for a single joint
            feedbackjoint.addString(tag);
            produceFeedback(i,dist_template2target,dist_candidate2target,feedbackjoint);
        }
    }

    /********************************************************/
    bool updateModule() override
    {
        LockGuard lg(mutex);

        //if we query the database
        if(opcPort.getOutputCount() > 0 && started)
        {
            //get skeleton
            getSkeleton();

            if(!skeletonIn.getTag().empty() && !skeletonTemplate.getTag().empty()
                    && first && use_robot_template == 1)
            {
                Vector p1=skeletonIn[KeyPointTag::shoulder_center]->getPoint();
                Vector c1=skeletonIn.getCoronal();
                Vector t1=skeletonIn.getTransverse();
                Vector s1=skeletonIn.getSagittal();
                Matrix Temp1 =zeros(4,4);
                Temp1.setSubcol(c1,0,0);
                Temp1.setSubcol(s1,0,1);
                Temp1.setSubcol(t1,0,2);
                Temp1.setSubcol(p1,0,3);
                Temp1(3,3)=1.0;

                Vector p2=skeletonTemplate[KeyPointTag::shoulder_center]->getPoint();
                Vector c2=skeletonTemplate.getCoronal();
                Vector t2=skeletonTemplate.getTransverse();
                Vector s2=skeletonTemplate.getSagittal();
                Matrix Temp2 =zeros(4,4);
                Temp2.setSubcol(c2,0,0);
                Temp2.setSubcol(s2,0,1);
                Temp2.setSubcol(t2,0,2);
                Temp2.setSubcol(p2,0,3);
                Temp2(3,3)=1.0;
                T2 =Temp1*SE3inv(Temp2);
                first=false;
            }

            skeletonIn.setTransformation(T);
            skeletonIn.update();
            skeletonIn.normalize(); //we normalize to avoid differences due to different physiques

            skeletonTemplate.setTransformation(T*T2);
            skeletonTemplate.update();
            skeletonTemplate.normalize();

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
                    if(action == exercise && confidence > action_threshold)
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
                    else if(action == "static")
                    {
                        Bottle &bList=outfeedback.addList();
                        Bottle &stat=bList.addList();
                        stat.addString("static");
                    }
                    else if(action == "random")
                    {
                        Bottle &bList=outfeedback.addList();
                        Bottle &random=bList.addList();
                        random.addString("random");
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
    int
    performFFT(const vector<double> &s, double &max)
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
