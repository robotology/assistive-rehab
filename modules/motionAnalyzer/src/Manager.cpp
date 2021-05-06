/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Manager.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <iostream>
#include <vector>
#include <list>
#include <iomanip>

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include "Manager.h"
#include "Metric.h"
#include "Processor.h"

#include "src/motionAnalyzer_IDL.h"

#include "matio.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace assistive_rehab;

/********************************************************/
vector<string> toStringVector(const Bottle &b, const string &key)
{
    vector<string> v;
    if(Bottle *bOut=b.find(key).asList())
    {
        for(size_t k=0; k<bOut->size(); k++)
            v.push_back(bOut->get(k).asString());
    }
    return v;
}

/********************************************************/
Vector toVector(const Bottle &b, const string &key)
{
    Vector v;
    if(Bottle *bOut=b.find(key).asList())
    {
        for(size_t k=0; k<bOut->size(); k++)
            v.push_back(bOut->get(k).asDouble());
    }
    return v;
}

/********************************************************/
Property toProp(const Bottle &b, const string &ex_tag)
{
    int duration=b.find("duration").asInt();
    double twarp=b.find("twarp").asDouble();
    Property pout;
    pout.put("duration",duration);
    pout.put("twarp",twarp);
    vector<string> joint_list=toStringVector(b,"joint_list");
    if (ex_tag==ExerciseTag::abduction_left || ex_tag==ExerciseTag::internal_rotation_left ||
            ex_tag==ExerciseTag::external_rotation_left)
    {
        Vector sx_thresh=toVector(b,"sx_thresh");
        Vector sy_thresh=toVector(b,"sy_thresh");
        Vector sz_thresh=toVector(b,"sz_thresh");
        Vector range_freq=toVector(b,"range_freq");
        Vector psd_thresh=toVector(b,"psd_thresh");
        Property &p=pout.addGroup("thresh");
        Property &pjoint=p.addGroup("joint");
        Property &psx=p.addGroup("sx");
        Property &psy=p.addGroup("sy");
        Property &psz=p.addGroup("sz");
        Property &freq=p.addGroup("freq");
        Property &psd=p.addGroup("psd");
        for(int i=0;i<joint_list.size();i++)
        {
            pjoint.put("joint_"+to_string(i),joint_list[i]);
            psx.put("sx_thresh_"+to_string(i),sx_thresh[i]);
            psy.put("sy_thresh_"+to_string(i),sy_thresh[i]);
            psz.put("sz_thresh_"+to_string(i),sz_thresh[i]);
            freq.put("range_freq_"+to_string(i),range_freq[i]);
            psd.put("psd_thresh_"+to_string(i),psd_thresh[i]);
        }
    }
    if (ex_tag==ExerciseTag::reaching_left)
    {
        Vector radius=toVector(b,"radius");
        Vector zscore=toVector(b,"zscore_thresh");
        Vector inliers=toVector(b,"inliers_thresh");
        Property &p=pout.addGroup("thresh");
        Property &pjoint=p.addGroup("joint");
        Property &pr=p.addGroup("radius");
        Property &pscore=p.addGroup("zscore_thresh");
        Property &pinliers=p.addGroup("inliers_thresh");
        for(int i=0;i<joint_list.size();i++)
        {
            pjoint.put("joint_"+to_string(i),joint_list[i]);
            pr.put("radius_"+to_string(i),radius[i]);
            pscore.put("zscore_thresh_"+to_string(i),zscore[i]);
            pinliers.put("inliers_thresh_"+to_string(i),inliers[i]);
        }
    }
    return pout;
}

/********************************************************/
Property Manager::loadFeedbackList(const Bottle &bExercise, const string &ex_tag)
{
    Property feedparams;
    Bottle &bFeedback=bExercise.findGroup("feedback");
    if(!bFeedback.isNull())
    {
        feedparams.clear();
        feedparams=toProp(bFeedback,ex_tag);
    }
    return feedparams;
}

/********************************************************/
Metric* Manager::loadMetricsList(const Bottle &bMetricEx, const string &metric_tag,
                                 const string &metric_type)
{
    if(!bMetricEx.isNull())
    {
        if(metric_type==MetricType::rom)
        {
            string tag_joint=bMetricEx.find("tag_joint").asString();
            Vector ref_dir=toVector(bMetricEx,"ref_dir");
            string ref_joint=bMetricEx.check("ref_joint", Value("")).asString();;
            string tag_plane=bMetricEx.find("tag_plane").asString();
            double minv=bMetricEx.find("min").asDouble();
            double maxv=bMetricEx.find("max").asDouble();
            Rom::RomParams rp={tag_joint,tag_plane,ref_dir,ref_joint,minv,maxv};
            return new Rom(metric_type,metric_tag,rp);
        }
        if(metric_type==MetricType::step)
        {
            num=toVector(bMetricEx,"num");
            den=toVector(bMetricEx,"den");
            double step_thresh=bMetricEx.find("step_thresh").asDouble();
            double step_window=bMetricEx.find("step_window").asDouble();
            double time_window=bMetricEx.find("time_window").asDouble();
            double minv=bMetricEx.find("min").asDouble();
            double maxv=bMetricEx.find("max").asDouble();
            Step::StepParams sp={num,den,step_thresh,step_window,time_window,minv,maxv};
            return new Step(metric_type,metric_tag,sp);
        }
        if(metric_type==MetricType::end_point)
        {
            string tag_joint=bMetricEx.find("tag_joint").asString();
            Vector ref_dir=toVector(bMetricEx,"ref_dir");
            string tag_plane=bMetricEx.find("tag_plane").asString();
            double minv=bMetricEx.find("min").asDouble();
            double maxv=bMetricEx.find("max").asDouble();
            Vector target=toVector(bMetricEx,"target");
            EndPoint::EndPointParams ep_params={tag_joint,tag_plane,ref_dir,minv,maxv,target};
            return new EndPoint(metric_type,metric_tag,ep_params);
        }
    }
    return NULL;
}

/********************************************************/
Exercise* Manager::loadExerciseList(const Bottle &bGeneral, const string &ex_tag)
{
    if(ex_tag==ExerciseTag::abduction_left || ex_tag==ExerciseTag::internal_rotation_left ||
            ex_tag==ExerciseTag::external_rotation_left)
    {
        return new RangeOfMotion(ex_tag);
    }
    if(ex_tag==ExerciseTag::reaching_left)
    {
        return new ReachingLeft();
    }
    if(ex_tag==ExerciseTag::tug)
    {
        int N=bGeneral.find("vel_estimator_N").asInt();
        double D=bGeneral.find("vel_estimator_D").asDouble();
        finishline_thresh=bGeneral.find("finish-line-thresh").asDouble();
        standing_thresh=bGeneral.find("standing-thresh").asDouble();
        double distance=bGeneral.check("distance",Value(6.0)).asDouble();
        double time_high=bGeneral.check("time-high",Value(10.0)).asDouble();
        double time_medium=bGeneral.check("time-medium",Value(20.0)).asDouble();
        lin_est_shoulder=new AWLinEstimator(N,D); //(16,0.01);
        Tug::TugParams tugp={finishline_thresh,standing_thresh,distance,time_high,time_medium};
        return new Tug(tugp);
    }
    return NULL;
}

/********************************************************/
bool Manager::loadMotionList(ResourceFinder &rf)
{
    lock_guard<mutex> lg(mtx);
    rf.setDefaultContext(this->rf->getContext().c_str());
    rf.setDefaultConfigFile(this->rf->find("from").asString().c_str());
    rf.configure(0,NULL);
    Bottle &bGeneral = rf.findGroup("general");
    if(!bGeneral.isNull())
    {
        if(Bottle *bExercises = bGeneral.find("exercises").asList())
        {
            int nexercises = bExercises->size();
            exercises.resize(nexercises);
            for(int i=0; i<nexercises; i++)
            {
                string ex_tag = bExercises->get(i).asString();
                Bottle &bExercise = rf.findGroup(ex_tag);
                if(!bExercise.isNull())
                {
                    Bottle &bGeneralEx = bExercise.findGroup("general");
                    exercises[i]=loadExerciseList(bGeneralEx, ex_tag);
                    //for each exercise we can evaluate different metrics
                    //check which metric has to be evaluated for this exercise
                    Bottle &bMetrics = bExercise.findGroup("metrics");
                    if(!bMetrics.isNull())
                    {
                        Bottle *bMetricTags = bMetrics.find("tag").asList();
                        Bottle *bMetricNumber = bMetrics.find("nmetrics").asList();
                        if(!bMetricTags->isNull() && !bMetricNumber->isNull())
                        {
                            for(int j=0; j<bMetricTags->size(); j++)
                            {
                                string metric_type = bMetricTags->get(j).asString();
                                int nmetrics = bMetricNumber->get(j).asInt();
                                for(int k=0; k<nmetrics; k++)
                                {
                                    string metric_tag = metric_type+"_"+to_string(k);
                                    Bottle &bMetricEx = bExercise.findGroup(metric_tag);
                                    Metric *m = loadMetricsList(bMetricEx, metric_tag, metric_type);
                                    exercises[i]->addMetric(m);
                                }
                            }
                        }
                    }
                    //each exercise can have a specific feedback
                    Property feedparams = loadFeedbackList(bExercise, ex_tag);
                    exercises[i]->setFeedbackParams(feedparams);
                }
                //add the exercise to the repertoire
                motion_repertoire.insert(pair<string,Exercise*>(ex_tag,exercises[i]));
                motion_repertoire[ex_tag]->print();
            }
        }
    }
    else
    {
        yError() << "Error in loading parameters. Stopping module!";
        return false;
    }
    return true;
}

/********************************************************/
bool Manager::setTemplateTag(const string &template_tag)
{
    lock_guard<mutex> lg(mtx);

    this->template_tag=template_tag;
    Bottle cmd,reply;
    cmd.clear();
    reply.clear();
    cmd.addString("setTemplateTag");
    cmd.addString(template_tag);
    dtwPort.write(cmd,reply);
    if(reply.get(0).asVocab()!=Vocab::encode("ok"))
    {
        yError() << "feedbackProducer could not load the template tag";
        return false;
    }
}

/********************************************************/
bool Manager::loadExercise(const string &exercise_tag)
{
    lock_guard<mutex> lg(mtx);
    if(motion_repertoire.count(exercise_tag))
    {
        curr_exercise=motion_repertoire.at(exercise_tag);
        yInfo()<<"Exercise to perform"<<curr_exercise->getName();
        vector<const Metric*> metrics=curr_exercise->getMetrics();
        processors.resize(metrics.size());
        for(int i=0; i<metrics.size(); i++)
        {
            string metric_tag=metrics[i]->getParams().find("name").asString();
            processors[i]=createProcessor(metric_tag,metrics[i]);
        }
        if(curr_exercise->getType()==ExerciseType::rehabilitation)
        {
            Bottle cmd,reply;
            cmd.addVocab(Vocab::encode("load"));
            cmd.addString(exercise_tag);
            actionPort.write(cmd,reply);
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError()<<"actionRecognizer could not load exercise tag" <<exercise_tag;
                return false;
            }

            cmd.clear();
            reply.clear();
            cmd.addString("setFeedbackThresh");
            cmd.addList().read(curr_exercise->getFeedbackThresholds());
            dtwPort.write(cmd,reply);
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError() << "feedbackProducer could not load the feedback thresholds";
                return false;
            }

            cmd.clear();
            reply.clear();
            cmd.addString("setJoints");
            Property feedparams=curr_exercise->getFeedbackParams();
            Bottle t=feedparams.findGroup("thresh");
            Bottle jnt=t.findGroup("joint");
            vector<string> joint_list;
            Bottle &jl = cmd.addList();
            yInfo()<<"Producing feedback for";
            for(int i=0;i<jnt.size()-1;i++)
            {
                Bottle *t1=jnt.get(i+1).asList();
                joint_list.push_back(t1->get(1).asString());
                cout<<joint_list[i]<<" ";
                jl.addString(joint_list[i]);
            }
            cout<<endl;
            yInfo() << cmd.toString();
            dtwPort.write(cmd,reply);
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError() << "feedbackProducer could not load the joint list";
                return false;
            }
        }

        return true;
    }
    else
    {
        yWarning() << "The selected exercise is not in the repertoire";
        yWarning() << "The available exercises can be listed using the listExercises command";
        return false;
    }
}
/********************************************************/
string Manager::getExercise()
{
    lock_guard<mutex> lg(mtx);
    if(curr_exercise!=NULL)
        return curr_exercise->getName();
    else
        return "";

}

/********************************************************/
vector<string> Manager::listExercises()
{
    lock_guard<mutex> lg(mtx);
    vector<string> reply;
    for (auto it=motion_repertoire.begin(); it!=motion_repertoire.end(); it++)
    {
        reply.push_back(it->first);
    }

    return reply;
}

/********************************************************/
vector<string> Manager::listMetricProps()
{
    lock_guard<mutex> lg(mtx);
    if(curr_metric!=NULL)
        return curr_metric->getProperties();
    else
        return vector<string>();
}

/********************************************************/
vector<string> Manager::listJoints()
{
    lock_guard<mutex> lg(mtx);
    vector<string> reply;
    if(curr_exercise!=NULL && curr_exercise->getType()==ExerciseType::rehabilitation)
    {
        Property feedparams=curr_exercise->getFeedbackParams();
        Bottle t=feedparams.findGroup("thresh");
        Bottle jnt=t.findGroup("joint");
        for(int i=0;i<jnt.size()-1;i++)
        {
            Bottle *t1=jnt.get(i+1).asList();
            reply.push_back(t1->get(1).asString());
        }
    }

    return reply;
}

/********************************************************/
bool Manager::selectSkel(const string &skel_tag)
{
    lock_guard<mutex> lg(mtx);

    this->skel_tag=skel_tag;
    yInfo() << "Analyzing skeleton " << this->skel_tag.c_str();

    if(curr_exercise->getType()==ExerciseType::rehabilitation)
    {
        //send tag to skeletonScaler
        Bottle cmd,reply;
        cmd.addVocab(Vocab::encode("tags"));
        cmd.addString(this->skel_tag);
        yInfo() << cmd.toString();
        actionPort.write(cmd,reply);

        cmd.clear();
        reply.clear();
        cmd.addString("setSkelTag");
        cmd.addString(this->skel_tag);
        dtwPort.write(cmd,reply);
        if(reply.get(0).asVocab()!=Vocab::encode("ok"))
        {
            yError() << "feedbackProducer could not load the skeleton tag";
            return false;
        }
    }
    return true;
}

/********************************************************/
bool Manager::setPart(const string &part)
{
    lock_guard<mutex> lg(mtx);

    yInfo() << "Moving" << part << "arm";
    Bottle cmd,reply;
    cmd.addString("loadModel");
    cmd.addString(part);
    actionPort.write(cmd,reply);
    if(reply.get(0).asVocab()!=Vocab::encode("ok"))
    {
        yError() << "Could not set part to actionRecognizer";
        return false;
    }

    cmd.clear();
    reply.clear();
    cmd.addString("setPart");
    cmd.addString(part);
    dtwPort.write(cmd,reply);
    if(reply.get(0).asVocab()!=Vocab::encode("ok"))
    {
        yError() << "Could not set part to feedbackProducer";
        return false;
    }

    return true;

}

/********************************************************/
bool Manager::mirrorTemplate(const bool robot_skeleton_mirror)
{
    lock_guard<mutex> lg(mtx);
    this->robot_skeleton_mirror=robot_skeleton_mirror;
    return true;
}

/********************************************************/
bool Manager::selectMetricProp(const string &prop_tag)
{
    lock_guard<mutex> lg(mtx);
    if(curr_exercise!=NULL)
    {
        this->prop_tag=prop_tag;
        yInfo()<<"Visualizing property"<<this->prop_tag;
        return true;
    }
    else
    {
        yWarning()<<"You need to select an exercise first";
        return false;
    }
}

/********************************************************/
string Manager::getCurrMetricProp()
{
    lock_guard<mutex> lg(mtx);
    return prop_tag;
}

/********************************************************/
vector<string> Manager::listMetrics()
{
    lock_guard<mutex> lg(mtx);
    if(curr_exercise!=NULL)
    {
        return curr_exercise->listMetrics();
    }
    else
    {
        yWarning()<<"You need to select an exercise first";
        return vector<string>();
    }
}

/********************************************************/
bool Manager::selectMetric(const string &metric_tag)
{
    lock_guard<mutex> lg(mtx);
    if(curr_exercise!=NULL)
    {
        curr_metric=curr_exercise->getCurrMetric(metric_tag);
        yInfo()<<"Analyzing"<<metric_tag;
        vector<string> available_props=curr_metric->getProperties();
        if(available_props.size()>0)
            prop_tag=available_props.back();

        if(curr_exercise->getType()==ExerciseType::rehabilitation)
        {
            Bottle cmd,reply;
            cmd.addString("setMetric");
            cmd.addString(metric_tag);
            dtwPort.write(cmd,reply);
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError()<<"feedbackProducer could not load the metric tag";
                return false;
            }

            Property currmet=curr_metric->getParams();
            cmd.clear();
            reply.clear();
            cmd.addString("setTarget");
            cmd.addList().read(currmet.find("target"));
            dtwPort.write(cmd,reply);
            yInfo() << cmd.toString();
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError() << "feedbackProducer could not load the target";
                return false;
            }
            return true;
        }
        return true;
    }
    else
    {
        yInfo()<<"You need to select the exercise first";
        return false;
    }
}

/********************************************************/
bool sendCmd(const RpcClient &port, const Bottle &cmd)
{
    Bottle reply;
    port.write(cmd,reply);
    if(reply.get(0).asVocab()!=Vocab::encode("ok"))
    {
        yError() << "failure from" << port.getName();
        return false;
    }
    return true;
}

/********************************************************/
bool Manager::runScaler()
{
    Property params=curr_exercise->getFeedbackParams();
    Bottle cmd;
    cmd.addVocab(Vocab::encode("load"));
    cmd.addString(template_tag + ".log");
    cmd.addString(rf->getContext());
    if(!sendCmd(scalerPort,cmd))
    {
        yError()<<"skeletonScaler could not load"<<template_tag;
        return false;
    }
    cmd.clear();
    cmd.addVocab(Vocab::encode("tags"));
    cmd.addString(skel_tag);
    if (!sendCmd(scalerPort,cmd))
    {
        yError()<<"skeletonScaler could not load"<<skel_tag;
        return false;
    }
    cmd.clear();
    cmd.addVocab(Vocab::encode("run"));
    cmd.addDouble(params.find("twarp").asDouble());
    if (!sendCmd(scalerPort,cmd))
    {
        yError()<<"Could not run skeletonScaler";
        return false;
    }
    return true;
}

/********************************************************/
bool Manager::runActionRecognizer(const Matrix &T)
{
    Property params=curr_exercise->getFeedbackParams();
    Bottle cmd;
    cmd.addString("setTransformation");
    cmd.addList().read(T);
    if (!sendCmd(actionPort,cmd))
    {
        yError()<<"actionRecognizer could not load transformation matrix";
        return false;
    }
    cmd.clear();
    cmd.addVocab(Vocab::encode("run"));
    cmd.addInt(params.find("duration").asInt());
    if (!sendCmd(actionPort,cmd))
    {
        yError()<<"Could not run actionRecognizer";
        return false;
    }
    return true;
}

/********************************************************/
bool Manager::runDtw(const Matrix &T)
{
    Bottle cmd;
    if (this->use_robot_template)
    {
        yInfo()<<"Using robot template";
        cmd.addString("setRobotTemplate");
        cmd.addInt(this->use_robot_template);
        cmd.addInt(this->robot_skeleton_mirror);
        if (!sendCmd(dtwPort,cmd))
        {
            yError()<<"feedbackProducer could not load robot template";
            return false;
        }
    }
    cmd.clear();
    cmd.addString("setTransformation");
    cmd.addList().read(T);
    if (!sendCmd(dtwPort,cmd))
    {
        yError()<<"feedbackProducer could not load transformation matrix";
        return false;
    }
    cmd.clear();
    cmd.addString("start");
    if (!sendCmd(dtwPort,cmd))
    {
        yError()<<"Could not run feedbackProducer";
        return false;
    }
    return true;
}

/********************************************************/
bool Manager::start(const bool use_robot_template)
{
    lock_guard<mutex> lg(mtx);                
    this->use_robot_template = use_robot_template;
    yInfo() << "Start!";
    starting=true;
    if(curr_exercise->getType()==ExerciseType::rehabilitation)
    {
        bool out=false;
        while(out==false)
        {
            getSkeleton();
            //we do not start if we haven't selected a skeleton tag
            if(skel_tag.empty() || !updated)
            {
                yWarning() << "Please select a proper skeleton tag";
                return false;
            }
            out=skeletonIn.update_planes();
            Time::yield();
        }
        Matrix T;
        for(int i=0; i<processors.size(); i++)
        {
            processors[i]->setInitialConf(skeletonIn,T);
        }
        if(this->use_robot_template == 0)
        {
            yInfo() << "Using pre-recorded template";
            if (!runScaler())
            {
                return false;
            }
        }
        if (runActionRecognizer(T) && runDtw(T))
        {
            tstart_session=Time::now()-tstart;
            starting=true;
            return true;
        }
        else
        {
            return false;
        }
    }
    for(int i=0; i<processors.size(); i++)
    {
        processors[i]->setStartingTime(Time::now());
    }
    tstart_session=Time::now()-tstart;
    return true;
}

/********************************************************/
bool Manager::stopFeedback()
{
    lock_guard<mutex> lg(mtx);

    yInfo() << "Stop feedback!";

    Bottle cmd,reply;
    cmd.addVocab(Vocab::encode("stop"));
    dtwPort.write(cmd,reply);
    actionPort.write(cmd,reply);
    if(reply.get(0).asVocab()==Vocab::encode("ok"))
    {
        return true;
    }
    return false;
}

/********************************************************/
bool Manager::stop()
{
    lock_guard<mutex> lg(mtx);
    if(starting)
    {
        yInfo()<<"stopping";
        if(curr_exercise->getType()==ExerciseType::rehabilitation)
        {
            Bottle cmd, reply;
            cmd.addVocab(Vocab::encode("stop"));
            dtwPort.write(cmd,reply);
            actionPort.write(cmd,reply);
            if(reply.get(0).asVocab()==Vocab::encode("ok") && starting)
            {
                if(use_robot_template == 0)
                {
                    yInfo()<<"Stopping skeletonScaler";
                    scalerPort.write(cmd,reply);
                    cmd.clear();
                    reply.clear();
                    cmd.addVocab(Vocab::encode("rot"));
                    cmd.addList().read(cameraposinit);
                    cmd.addList().read(focalpointinit);
                    scalerPort.write(cmd, reply);
                }
            }
        }
        tend_session=Time::now()-tstart;
        writeMatio();
        reset();
        return true;
    }
    return true;
}

/****************************************************************/
void Manager::reset()
{
    for(int i=0; i<processors.size(); i++)
    {
        processors[i]->reset();
    }
    nsession++;
    starting=false;
    skel_tag="";
    bResult.clear();
    curr_exercise=NULL;
    curr_metric=NULL;
    standing=false;
    state=State::idle;
}

/****************************************************************/
void Manager::writeMatio()
{
    // Use MATIO to write the results in a .mat file
    string filename_report=out_folder+"/user-"+skeletonIn.getTag()+
            "-"+curr_exercise->getName()+"-"+to_string(nsession)+".mat";
    mat_t *matfp=Mat_CreateVer(filename_report.c_str(),NULL,MAT_FT_MAT5);
    if (matfp==NULL)
        yError()<<"Error creating MAT file";
    yInfo() << "Writing to file";
    if(writeKeypointsToFile(matfp))
    {
        time_samples.clear();
        all_keypoints.clear();
    }
    else
        yError() << "Could not save to file";

    yInfo() << "Keypoints saved to file" << filename_report.c_str();
    Mat_Close(matfp);
}

/****************************************************************/
Property Manager::publishState()
{
    Property p;
    if (bResult.size()>0)
    {
        for (size_t i=0;i<bResult.size();i++)
        {
            p.put(processors[i]->getProcessedMetric(),bResult.get(i));
        }
    }
    if (state==State::idle)
    {
        p.put("human-state", "idle");
    }
    else if (state==State::crossed)
    {
        p.put("human-state", "crossed");
    }
    else if (state==State::sitting)
    {
        p.put("human-state", "sitting");
    }
    else if (state==State::standing)
    {
        p.put("human-state", "standing");
    }
    if (curr_exercise!=NULL)
    {
        Bottle bEx;
        bEx.addList().read(curr_exercise->publish());
        p.put("exercise",bEx.get(0));
    }
    return p;
}

/********************************************************/
Property Manager::getState()
{
    lock_guard<mutex> lg(mtx);
    return publishState();
}

/********************************************************/
bool Manager::isStanding()
{
    yInfo()<<"shoulder height speed"<<shoulder_center_height_vel;
    standing=(shoulder_center_height_vel>standing_thresh);
    return standing;
}

/********************************************************/
bool Manager::isSitting()
{
    yInfo()<<"shoulder height speed"<<shoulder_center_height_vel;
    return (shoulder_center_height_vel<-standing_thresh);
}

/********************************************************/
bool Manager::hasCrossedFinishLine()
{
    Vector foot_right=skeletonIn[KeyPointTag::ankle_right]->getPoint();
    Vector foot_left=skeletonIn[KeyPointTag::ankle_left]->getPoint();

    Vector lp_world(3);
    lp_world[0]=line_pose[0];
    lp_world[1]=line_pose[1];
    lp_world[2]=line_pose[2];

    Vector line_ori(4);
    line_ori[0]=line_pose[3];
    line_ori[1]=line_pose[4];
    line_ori[2]=line_pose[5];
    line_ori[3]=line_pose[6];
    Matrix lOri=axis2dcm(line_ori);
    Vector line_x=lOri.getCol(0);
    line_x.pop_back();

    Vector fr_lp=lp_world-foot_right;
    Vector fl_lp=lp_world-foot_left;
    Vector pline=lp_world+line_x;
    Vector v1=lp_world-pline;
    double dist_fr_line=norm(cross(v1,fr_lp))/norm(v1);
    double dist_fl_line=norm(cross(v1,fl_lp))/norm(v1);
    yInfo()<<"dist foot right line"<<dist_fr_line;
    yInfo()<<"dist foot left line"<<dist_fl_line;

    return (dist_fr_line<finishline_thresh && dist_fl_line<finishline_thresh);
}

/********************************************************/
bool Manager::setLinePose(const vector<double> &line_pose)
{
    lock_guard<mutex> lg(mtx);
    this->line_pose=line_pose;
    return true;
}

/********************************************************/
void Manager::getSkeleton()
{
    //ask for the property id
    Bottle cmd,reply;
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
                                    if(!tag.empty())
                                    {
                                        if(prop.check("tag") && tag==skel_tag)
                                        {
                                            Skeleton* skeleton = skeleton_factory(prop);
                                            skeletonIn.update(skeleton->toProperty());
                                            if(skeleton->update_planes() && starting)
                                            {
                                                vector<pair<string,Vector>> keyps=skeletonIn.get_unordered();
                                                all_keypoints.push_back(keyps);
                                            }
                                            if(skeletonIn[KeyPointTag::shoulder_center]->isUpdated() && curr_exercise->getName()==ExerciseTag::tug)
                                            {
                                                Vector shoulder_center=skeletonIn[KeyPointTag::shoulder_center]->getPoint();
                                                AWPolyElement el(shoulder_center,Time::now());
                                                shoulder_center_height_vel=lin_est_shoulder->estimate(el)[2];
                                            }
                                            updated=true;
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
}

/********************************************************/
void Manager::updateState()
{
    if (!standing)
    {
        if (isStanding())
        {
            state=State::standing;
        }
    }
    else if (hasCrossedFinishLine())
    {
        state=State::crossed;
    }
    else if (isSitting())
    {
        state=State::sitting;
    }
}

/********************************************************/
void Manager::estimate()
{
    Bottle &scopebottleout=scopePort.prepare();
    scopebottleout.clear();
    bResult.clear();
    for(int i=0; i<processors.size(); i++)
    {
        processors[i]->update(skeletonIn);
        processors[i]->estimate();
        Property result=processors[i]->getResult();
        bResult.addList().read(result);
        if(result.check(prop_tag) &&
                processors[i]->getProcessedMetric()==curr_metric->getParams().find("name").asString())
        {
            double res=result.find(prop_tag).asDouble();
            scopebottleout.addDouble(res);
        }
    }
    scopePort.write();
}

/********************************************************/
bool Manager::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/********************************************************/
bool Manager::configure(ResourceFinder &rf)
{
    this->rf = &rf;
    string moduleName = rf.check("name", Value("motionAnalyzer")).asString();
    setName(moduleName.c_str());

    opcPort.open(("/" + getName() + "/opc").c_str());
    scopePort.open(("/" + getName() + "/scope").c_str());
    scalerPort.open(("/" + getName() + "/scaler:cmd").c_str());
    dtwPort.open(("/" + getName() + "/dtw:cmd").c_str());
    actionPort.open(("/" + getName() + "/action:cmd").c_str());
    rpcPort.open(("/" + getName() + "/cmd").c_str());
    attach(rpcPort);

    cameraposinit.resize(3);
    focalpointinit.resize(3);

    if(!loadMotionList(rf))
        return false;

    out_folder=rf.getHomeContextPath();
    tstart=Time::now();
    nsession=0;
    finishedSession=false;
    starting=false;
    skel_tag="";
    prop_tag="";
    curr_exercise=NULL;
    curr_metric=NULL;
    state=State::idle;
    standing=false;
    return true;
}

/********************************************************/
bool Manager::interruptModule()
{
    opcPort.interrupt();
    scopePort.interrupt();
    scalerPort.interrupt();
    dtwPort.interrupt();
    actionPort.interrupt();
    rpcPort.interrupt();
    yInfo() << "Interrupted module";
    return true;
}

/********************************************************/
bool Manager::close()
{
    for(int i=0; i<exercises.size(); i++)
    {
        if (exercises[i]->getName()==ExerciseTag::tug)
        {
            delete lin_est_shoulder;
        }
        delete exercises[i];
    }
    for(int i=0; i<processors.size(); i++)
    {
        delete processors[i];
    }
    yInfo() << "Freed memory";

    opcPort.close();
    scopePort.close();
    scalerPort.close();
    dtwPort.close();
    actionPort.close();
    rpcPort.close();
    yInfo() << "Closed ports";
    return true;
}

/********************************************************/
double Manager::getPeriod()
{
    return 0.1;
}

/********************************************************/
bool Manager::updateModule()
{
    lock_guard<mutex> lg(mtx);
    //if we query the database
    if(opcPort.getOutputCount()>0)
    {
        //get skeleton and normalize
        getSkeleton();
        //if no metric has been defined we do not analyze motion
        if(starting)
        {
            if(curr_exercise!=NULL && curr_metric!=NULL)
            {
                if(updated)
                {
                    //update time array
                    time_samples.push_back(Time::now()-tstart);
                    if (curr_exercise->getName()==ExerciseTag::tug)
                    {
                        updateState();
                    }
                    estimate();
                }
            }
            else
                yInfo() << "Please specify metric";
        }
    }
    return true;
}

/********************************************************/
bool Manager::writeStructToMat(const string& name, const vector< vector< pair<string,Vector> > >& keypoints_skel, mat_t *matfp)
{
    int numKeypoints = skeletonIn.getNumKeyPoints();
    const char *fields[numKeypoints];
    for(int i=0; i<numKeypoints; i++)
    {
        fields[i]=keypoints_skel[0][i].first.c_str();
    }

    size_t dim_struct[2] = {1,1};

    size_t nSamples = keypoints_skel.size()-1;

    matvar_t *matvar = Mat_VarCreateStruct(name.c_str(),2,dim_struct,fields,numKeypoints);
    if(matvar != NULL)
    {
        vector<double> field_vector;
        field_vector.resize(3*(nSamples));

        size_t dims_field[2] = {nSamples,3};

    //    print(keypoints_skel);

        for(int i=0; i<numKeypoints; i++)
        {
            for(int j=0; j<nSamples; j++)
            {
                field_vector[j] = keypoints_skel[j][i].second[0];
                field_vector[j+nSamples] = keypoints_skel[j][i].second[1];
                field_vector[j+2*nSamples] = keypoints_skel[j][i].second[2];
            }

            matvar_t *field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field,field_vector.data(),MAT_F_GLOBAL);
            Mat_VarSetStructFieldByName(matvar, fields[i], 0, field);
        }

        Mat_VarWrite(matfp,matvar,MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
    }
    else
        return false;

    return true;

}

/********************************************************/
bool Manager::writeStructToMat(const string& name, const Exercise* ex, mat_t *matfp)
{
    vector<const Metric*> metrics=ex->getMetrics();
    size_t numMetrics=metrics.size();
    const char *fields[2]={"name","metrics"};

    //main struct for exercise
    size_t dim_struct[2]={1,1};
    matvar_t *matvar=Mat_VarCreateStruct(name.c_str(),2,dim_struct,fields,2);
    if(matvar==NULL)
    {
        yError()<<"Could not create exercise structure";
        return false;
    }
    string currex_name=curr_exercise->getName();
    size_t dim_name[2]={1,currex_name.size()};
    char *exname=new char[currex_name.size()+1];
    strcpy(exname, currex_name.c_str());
    matvar_t *ex_matvar=Mat_VarCreate(fields[0],MAT_C_CHAR,MAT_T_UTF8,2,dim_name,exname,0);
    Mat_VarSetStructFieldByName(matvar,fields[0],0,ex_matvar);
    delete [] exname;

    size_t dim_struct_metrics[2]={1,1};
    const char *fields_metrics[numMetrics];
    vector<string> fieldnames(numMetrics,"");
    for(size_t i=0; i<numMetrics; i++)
    {
        Property params=metrics[i]->getParams();
        fieldnames[i]=params.find("name").asString();
        fields_metrics[i]=fieldnames[i].c_str();
    }
    matvar_t *met_matvar=Mat_VarCreateStruct("metrics",2,dim_struct_metrics,fields_metrics,numMetrics);
    if(met_matvar==NULL)
    {
        yError()<<"Could not create exercise structure";
        return false;
    }

    for(size_t i=0; i<numMetrics; i++)
    {
        matvar_t *submatvar=writeStructToMat(metrics[i]);
        Mat_VarSetStructFieldByName(met_matvar,fields_metrics[i],0,submatvar);
    }

    Mat_VarSetStructFieldByName(matvar,fields[1],0,met_matvar);
    Mat_VarWrite(matfp,matvar,MAT_COMPRESSION_NONE);
    Mat_VarFree(matvar);

    return true;
}

/********************************************************/
void Manager::createSubfield(matvar_t *submatvar, double *val, size_t *dims, const char *name)
{
    matvar_t *subfield;
    subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,&val,MAT_F_DONT_COPY_DATA);
    Mat_VarSetStructFieldByName(submatvar,name,0,subfield);
}

/********************************************************/
void Manager::createSubfield(matvar_t *submatvar, const string &val_str, size_t *dims, const char *name)
{
    char *val=new char[val_str.length()+1];
    strcpy(val, val_str.c_str());
    matvar_t *subfield;
    subfield=Mat_VarCreate(NULL,MAT_C_CHAR,MAT_T_UTF8,2,dims,val,0);
    Mat_VarSetStructFieldByName(submatvar,name,0,subfield);
    delete [] val;
}

/********************************************************/
matvar_t* Manager::createRomField(const Property &params)
{
    string metric_name=params.find("name").asString();
    int numFields=8;
    const char *subfields[numFields]={"tag_joint","ref_joint","ref_dir","tag_plane","max","min","tstart","tend"};
    size_t dim_struct[2]={1,1};
    matvar_t* submatvar;
    submatvar=Mat_VarCreateStruct(metric_name.c_str(),2,dim_struct,subfields,numFields);
    string joint_met=params.find("tag_joint").asString();
    size_t dims_field_joint[2]={1,joint_met.size()};
    createSubfield(submatvar,joint_met,dims_field_joint,subfields[0]);
    string ref_joint=params.find("ref_joint").asString();
    size_t dims_field_refjoint[2]={1,ref_joint.size()};
    createSubfield(submatvar,ref_joint,dims_field_refjoint,subfields[1]);
    size_t dims_field_dir[2]={1,3};
    Bottle *bRef=params.find("ref_dir").asList();
    Vector ref_met(3,0.0);
    ref_met[0]=bRef->get(0).asDouble();
    ref_met[1]=bRef->get(1).asDouble();
    ref_met[2]=bRef->get(2).asDouble();
    createSubfield(submatvar,ref_met.data(),dims_field_dir,subfields[2]);
    string tag_plane=params.find("tag_plane").asString();
    size_t dims_field_tagplane[2]={1,tag_plane.size()};
    createSubfield(submatvar,tag_plane,dims_field_tagplane,subfields[3]);
    size_t dims[2]={1,1};
    double max_val=params.find("max").asDouble();
    double min_val=params.find("min").asDouble();
    createSubfield(submatvar,&max_val,dims,subfields[4]);
    createSubfield(submatvar,&min_val,dims,subfields[5]);
    createSubfield(submatvar,&tstart_session,dims,subfields[6]);
    createSubfield(submatvar,&tend_session,dims,subfields[7]);
    return submatvar;
}

/********************************************************/
matvar_t* Manager::createStepField(const Property &params)
{
    string metric_name=params.find("name").asString();
    int numFields=7;
    const char *subfields[numFields]={"num","den","max","min","tstart","tend","step_thresh"};
    size_t dim_struct[2]={1,1};
    matvar_t* submatvar;
    submatvar=Mat_VarCreateStruct(metric_name.c_str(),2,dim_struct,subfields,numFields);
    size_t dims_field_num[2]={1,num.size()};
    size_t dims_field_den[2]={1,den.size()};
    createSubfield(submatvar,num.data(),dims_field_num,subfields[0]);
    createSubfield(submatvar,den.data(),dims_field_den,subfields[1]);
    size_t dims[2]={1,1};
    double max_val=params.find("max").asDouble();
    double min_val=params.find("min").asDouble();
    double step_thresh=params.find("step_thresh").asDouble();
    createSubfield(submatvar,&max_val,dims,subfields[2]);
    createSubfield(submatvar,&min_val,dims,subfields[3]);
    createSubfield(submatvar,&tstart_session,dims,subfields[4]);
    createSubfield(submatvar,&tend_session,dims,subfields[5]);
    createSubfield(submatvar,&step_thresh,dims,subfields[6]);
    return submatvar;
}

/********************************************************/
matvar_t* Manager::createEpField(const Property &params)
{
    string metric_name=params.find("name").asString();
    matvar_t* submatvar;
    int numFields=8;
    const char *subfields[numFields]={"tag_joint","ref_dir","tag_plane","max","min","target","tstart","tend"};
    size_t dim_struct[2]={1,1};
    submatvar=Mat_VarCreateStruct(metric_name.c_str(),2,dim_struct,subfields,numFields);
    string joint_met=params.find("tag_joint").asString();
    size_t dims_field_joint[2]={1,joint_met.size()};
    createSubfield(submatvar,joint_met,dims_field_joint,subfields[0]);
    size_t dims_field_dir[2]={1,3};
    Bottle *bRef=params.find("ref_dir").asList();
    Vector ref_met(3,0.0);
    ref_met[0]=bRef->get(0).asDouble();
    ref_met[1]=bRef->get(1).asDouble();
    ref_met[2]=bRef->get(2).asDouble();
    createSubfield(submatvar,ref_met.data(),dims_field_dir,subfields[1]);
    string tag_plane=params.find("tag_plane").asString();
    size_t dims_field_tagplane[2]={1,tag_plane.size()};
    createSubfield(submatvar,tag_plane,dims_field_tagplane,subfields[2]);
    size_t dims[2]={1,1};
    double max_val=params.find("max").asDouble();
    double min_val=params.find("min").asDouble();
    createSubfield(submatvar,&max_val,dims,subfields[3]);
    createSubfield(submatvar,&min_val,dims,subfields[4]);
    size_t dims_field_target[2]={1,3};
    Bottle *bTarget=params.find("target").asList();
    Vector target(3,0.0);
    target[0]=bTarget->get(0).asDouble();
    target[1]=bTarget->get(1).asDouble();
    target[2]=bTarget->get(2).asDouble();
    createSubfield(submatvar,target.data(),dims_field_target,subfields[5]);
    createSubfield(submatvar,&tstart_session,dims,subfields[6]);
    createSubfield(submatvar,&tend_session,dims,subfields[7]);
    return submatvar;
}

/********************************************************/
matvar_t * Manager::writeStructToMat(const Metric* m)
{
    matvar_t* submatvar=NULL;
    Property params=m->getParams();
    string metric_type=params.find("type").asString();
    if(metric_type==MetricType::rom)
    {
        submatvar=createRomField(params);
    }
    if(metric_type==MetricType::step)
    {
        submatvar=createStepField(params);
    }
    if(metric_type==MetricType::end_point)
    {
        submatvar=createEpField(params);
    }
    return submatvar;
}

/********************************************************/
void Manager::print(const vector< vector< pair<string,Vector> > >& keypoints_skel)
{
    for(int i=0; i<keypoints_skel[0].size(); i++)
    {
        for(int j=0; j<keypoints_skel.size(); j++)
        {
            cout << keypoints_skel[j][i].first << " " << keypoints_skel[j][i].second[0] << " "
                 << keypoints_skel[j][i].second[1] << " " << keypoints_skel[j][i].second[2] << endl;
        }
    }
    cout << endl;
}

/********************************************************/
bool Manager::writeKeypointsToFile(mat_t *matfp)
{
    // Use MATIO to write the results in a .mat file

    //Save time samples
    size_t nSamples=time_samples.size();
    size_t dims[2]={nSamples,1};
    matvar_t *matvar=Mat_VarCreate("Time_samples",MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims,time_samples.data(),0);
    if(matvar!=NULL)
    {
        Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_NONE);
        Mat_VarFree(matvar);
    }
    else
    {
        yError() << "Could not save time samples.. the file will not be saved";
        return false;
    }

//    if(all_keypoints.size() != 0)
//        print(all_keypoints);

    //Save keypoints
    if(!writeStructToMat("Keypoints",all_keypoints,matfp))
    {
        yError() << "Could not save keypoints.. the file will not be saved";
        return false;
    }

    //Save exercise
    if(!writeStructToMat("Exercise",curr_exercise,matfp))
    {
        yError() << "Could not save exercise.. the file will not be saved";
        return false;
    }

    return true;
}
