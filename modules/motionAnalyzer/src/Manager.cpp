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

bool Manager::loadMotionList(ResourceFinder &rf)
{
    lock_guard<mutex> lg(mtx);

    rf.setVerbose();
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
                    Bottle &bGeneral=bExercise.findGroup("general");
                    if(!bGeneral.isNull())
                    {
                        string type=bGeneral.find("type").asString();
                        if(ex_tag==ExerciseTag::abduction_left ||
                                ex_tag==ExerciseTag::internal_rotation_left ||
                                ex_tag==ExerciseTag::external_rotation_left)
                        {
                            exercises[i]=new RangeOfMotion(ex_tag);
                        }
                        if(ex_tag==ExerciseTag::reaching_left)
                        {
                            exercises[i]=new ReachingLeft();
                        }
                        if(ex_tag==ExerciseTag::tug)
                        {
                            exercises[i]=new Tug();
                        }
                    }

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
                                string metric_type=bMetricTags->get(j).asString();
                                int nmetrics=bMetricNumber->get(j).asInt();
                                for(int k=0; k<nmetrics; k++)
                                {                                   
                                    string metric_tag=metric_type+"_"+to_string(k);
                                    Bottle &bMetricEx=bExercise.findGroup(metric_tag);
                                    if(!bMetricEx.isNull())
                                    {
                                        if(metric_type==MetricType::rom)
                                        {
                                            string tag_joint = bMetricEx.find("tag_joint").asString();
                                            Vector ref_dir(3,0.0);
                                            if(Bottle *bRefDir = bMetricEx.find("ref_dir").asList())
                                            {
                                                ref_dir[0] = bRefDir->get(0).asDouble();
                                                ref_dir[1] = bRefDir->get(1).asDouble();
                                                ref_dir[2] = bRefDir->get(2).asDouble();
                                            }
                                            string ref_joint=bMetricEx.check("ref_joint", Value("")).asString();;
                                            string tag_plane=bMetricEx.find("tag_plane").asString();
                                            double minv=bMetricEx.find("min").asDouble();
                                            double maxv=bMetricEx.find("max").asDouble();

                                            Metric *rom;
                                            rom=new Rom(metric_type,metric_tag,tag_joint,tag_plane,ref_dir,ref_joint,minv,maxv);
                                            exercises[i]->addMetric(rom);
                                        }                                      
                                        if(metric_type==MetricType::step)
                                        {
                                            Bottle *bNum = bMetricEx.find("num").asList();
                                            Bottle *bDen = bMetricEx.find("den").asList();
                                            Vector num(6,0.0),den(6,0.0);
                                            if(!bNum->isNull() && !bDen->isNull())
                                            {
                                                num[0] = bNum->get(0).asDouble();
                                                num[1] = bNum->get(1).asDouble();
                                                num[2] = bNum->get(2).asDouble();
                                                num[3] = bNum->get(3).asDouble();
                                                num[4] = bNum->get(4).asDouble();
                                                num[5] = bNum->get(5).asDouble();

                                                den[0] = bDen->get(0).asDouble();
                                                den[1] = bDen->get(1).asDouble();
                                                den[2] = bDen->get(2).asDouble();
                                                den[3] = bDen->get(3).asDouble();
                                                den[4] = bDen->get(4).asDouble();
                                                den[5] = bDen->get(5).asDouble();
                                            }
                                            double minv=bMetricEx.find("min").asDouble();
                                            double maxv=bMetricEx.find("max").asDouble();
                                            Metric *step;
                                            step=new Step(metric_type,metric_tag,num,den,minv,maxv);
                                            exercises[i]->addMetric(step);
                                        }
                                        if(metric_type==MetricType::end_point)
                                        {
                                            string tag_joint=bMetricEx.find("tag_joint").asString();
                                            Vector ref_dir(3,0.0);
                                            if(Bottle *bRefDir = bMetricEx.find("ref_dir").asList())
                                            {
                                                ref_dir[0] = bRefDir->get(0).asDouble();
                                                ref_dir[1] = bRefDir->get(1).asDouble();
                                                ref_dir[2] = bRefDir->get(2).asDouble();
                                            }
                                            string tag_plane=bMetricEx.find("tag_plane").asString();
                                            double minv=bMetricEx.find("min").asDouble();
                                            double maxv=bMetricEx.find("max").asDouble();
                                            Vector target(3,0.0);
                                            if(Bottle *bTarget=bMetricEx.find("target").asList())
                                            {
                                                target[0]=bTarget->get(0).asDouble();
                                                target[1]=bTarget->get(1).asDouble();
                                                target[2]=bTarget->get(2).asDouble();
                                            }
                                            Metric *ep;
                                            ep=new EndPoint(metric_type,metric_tag,tag_joint,tag_plane,ref_dir,minv,maxv,target);
                                            exercises[i]->addMetric(ep);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    //each exercise can have a specific feedback
                    Bottle &bFeedback=bExercise.findGroup("feedback");
                    if(!bFeedback.isNull())
                    {
                        Property feedparams;
                        if(ex_tag==ExerciseTag::abduction_left ||
                                ex_tag==ExerciseTag::internal_rotation_left ||
                                ex_tag==ExerciseTag::external_rotation_left)
                        {
                            int duration=bFeedback.find("duration").asInt();
                            double twarp=bFeedback.find("twarp").asDouble();
                            vector<string> joint_list;
                            if(Bottle *bJointList=bFeedback.find("joint_list").asList())
                            {
                                for(size_t k=0; k<bJointList->size(); k++)
                                    joint_list.push_back(bJointList->get(k).asString());
                            }
                            Vector sx_thresh;
                            if(Bottle *bSxThresh=bFeedback.find("sx_thresh").asList())
                            {
                                for(size_t k=0; k<bSxThresh->size(); k++)
                                    sx_thresh.push_back(bSxThresh->get(k).asDouble());
                            }
                            Vector sy_thresh;
                            if(Bottle *bSyThresh=bFeedback.find("sy_thresh").asList())
                            {
                                for(size_t k=0; k<bSyThresh->size(); k++)
                                    sy_thresh.push_back(bSyThresh->get(k).asDouble());
                            }
                            Vector sz_thresh;
                            if(Bottle *bSzThresh=bFeedback.find("sz_thresh").asList())
                            {
                                for(size_t k=0; k<bSzThresh->size(); k++)
                                    sz_thresh.push_back(bSzThresh->get(k).asDouble());
                            }
                            Vector range_freq;
                            if(Bottle *bFreqThresh=bFeedback.find("range_freq").asList())
                            {
                                for(size_t k=0; k<bFreqThresh->size(); k++)
                                    range_freq.push_back(bFreqThresh->get(k).asInt());
                            }
                            Vector psd_thresh;
                            if(Bottle *bPsdThresh=bFeedback.find("psd_thresh").asList())
                            {
                                for(size_t k=0; k<bPsdThresh->size(); k++)
                                    psd_thresh.push_back(bPsdThresh->get(k).asDouble());
                            }

                            feedparams.clear();
                            feedparams.put("duration",duration);
                            feedparams.put("twarp",twarp);
                            Property &p=feedparams.addGroup("thresh");
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
                        if(ex_tag==ExerciseTag::reaching_left)
                        {
                            int duration=bFeedback.find("duration").asInt();
                            double twarp=bFeedback.find("twarp").asDouble();
                            vector<string> joint_list;
                            if(Bottle *bJointList=bFeedback.find("joint_list").asList())
                            {
                                for(size_t k=0; k<bJointList->size(); k++)
                                    joint_list.push_back(bJointList->get(k).asString());
                            }
                            Vector radius;
                            if(Bottle *bRadius=bFeedback.find("radius").asList())
                            {
                                for(size_t k=0; k<bRadius->size(); k++)
                                    radius.push_back(bRadius->get(k).asDouble());
                            }
                            Vector zscore;
                            if(Bottle *bZscore=bFeedback.find("zscore_thresh").asList())
                            {
                                for(size_t k=0; k<bZscore->size(); k++)
                                    zscore.push_back(bZscore->get(k).asDouble());
                            }
                            Vector inliers;
                            if(Bottle *bInliersThresh=bFeedback.find("inliers_thresh").asList())
                            {
                                for(size_t k=0; k<bInliersThresh->size(); k++)
                                    inliers.push_back(bInliersThresh->get(k).asDouble());
                            }

                            feedparams.clear();
                            feedparams.put("duration",duration);
                            feedparams.put("twarp",twarp);
                            Property &p=feedparams.addGroup("thresh");
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
                        exercises[i]->setFeedbackParams(feedparams);
                    }
                    //add the exercise to the repertoire
                    motion_repertoire.insert(pair<string,Exercise*>(ex_tag,exercises[i]));
                    motion_repertoire[ex_tag]->print();
                }
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

#ifndef NDEBUG
    yInfo() << "Debugging";
    {
        double t1 = Time::now();
        while( (Time::now()-t1) < 15.0 )
        {
            //do nothing
            yInfo() << "Waiting to start";
        }
        return start();
    }
#endif

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
bool Manager::start(const bool use_robot_template)
{
    lock_guard<mutex> lg(mtx);
                
    this->use_robot_template = use_robot_template;
    yInfo() << "Start!";
	
    if (!starting)
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
    }

    if(curr_exercise->getType()==ExerciseType::rehabilitation)
    {
        Matrix T;
        for(int i=0; i<processors.size(); i++)
        {
            processors[i]->setInitialConf(skeletonIn,T);
        }

        Property params=curr_exercise->getFeedbackParams();
        Bottle cmd,reply;
        if(this->use_robot_template == 0)
        {
            yInfo() << "Using pre-recorded template";
            cmd.clear();
            reply.clear();
            cmd.addVocab(Vocab::encode("load"));
            string file=template_tag + ".log";
            cmd.addString(file);
            string context=rf->getContext();
            cmd.addString(context);
            scalerPort.write(cmd,reply);
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError() << "skeletonScaler could not load" << file << "file";
                return false;
            }

            cmd.clear();
            reply.clear();
            cmd.addVocab(Vocab::encode("tags"));
            cmd.addString(skel_tag);
            scalerPort.write(cmd,reply);
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError() << "skeletonScaler could not select" << skel_tag;
                return false;
            }

            cmd.clear();
            reply.clear();
            cmd.addVocab(Vocab::encode("run"));
            cmd.addDouble(params.find("twarp").asDouble());
            scalerPort.write(cmd,reply);
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError() << "Could not run skeletonScaler";
                return false;
            }
        }
        else
        {
            yInfo() << "Using robot template";
            cmd.clear();
            reply.clear();
            cmd.addString("setRobotTemplate");
            cmd.addInt(this->use_robot_template);
            cmd.addInt(this->robot_skeleton_mirror);
            dtwPort.write(cmd,reply);
            if(reply.get(0).asVocab()!=Vocab::encode("ok"))
            {
                yError() << "Could not set robot template to feedbackProducer";
                return false;
            }
        }

        reply.clear();
        cmd.clear();
        cmd.addString("setTransformation");
        cmd.addList().read(T);
        yDebug() << T.toString();
        dtwPort.write(cmd,reply);
        if(!reply.get(0).asVocab()==Vocab::encode("ok"))
        {
            yError() << "Could not set tranformation in feedbackProducer";
            return false;
        }

        reply.clear();
        actionPort.write(cmd,reply);
        if(!reply.get(0).asVocab()==Vocab::encode("ok"))
        {
            yError() << "Could not set tranformation in actionRecognizer";
            return false;
        }

        cmd.clear();
        reply.clear();
        cmd.addVocab(Vocab::encode("run"));
        cmd.addInt(params.find("duration").asInt());
        actionPort.write(cmd,reply);
        if(reply.get(0).asVocab()!=Vocab::encode("ok"))
        {
            yError() << "Could not run actionRecognizer";
            return false;
        }

        reply.clear();
        cmd.clear();
        cmd.addString("start");
        dtwPort.write(cmd,reply);
        if(reply.get(0).asVocab()==Vocab::encode("ok"))
        {
            tstart_session=Time::now()-tstart;
            starting=true;
            return true;
        }
        else
        {
            yError() << "Could not run feedbackProducer";
            return false;
        }
    }

    if(frozen)
    {
        frozen=false;
        if(!starting)
        {
            starting=true;
            tstart_session=Time::now()-tstart;
            return true;
        }
    }

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

        nsession++;
        starting=false;
        skel_tag="";

        return true;
    }
    return true;
}

/********************************************************/
bool Manager::freeze()
{
    lock_guard<mutex> lg(mtx);
    yInfo()<<"Freezing";
    frozen=true;
    return true;
}

/********************************************************/
bool Manager::isStanding(const double standing_thresh)
{
    lock_guard<mutex> lg(mtx);
    yInfo()<<"shoulder height speed"<<shoulder_center_height_vel;
    return (shoulder_center_height_vel>standing_thresh);
}

/********************************************************/
bool Manager::isSitting(const double standing_thresh)
{
    lock_guard<mutex> lg(mtx);
    yInfo()<<"shoulder height speed"<<shoulder_center_height_vel;
    return (shoulder_center_height_vel<-standing_thresh);
}

/********************************************************/
bool Manager::hasCrossedFinishLine(const double finishline_thresh)
{
    lock_guard<mutex> lg(mtx);
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

//    yInfo() << "Query opc: " << cmd.toString();
    opcPort.write(cmd, reply);
//    yInfo() << "Reply from opc:" << reply.toString();

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

//                            yInfo() << "Command sent to the port: " << cmd.toString();
                            opcPort.write(cmd, replyProp);
//                            yInfo() << "Reply from opc:" << replyProp.toString();

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
                                            if(skeleton->update_planes())
                                            {
                                                vector<pair<string,Vector>> keyps=skeletonIn.get_unordered();
                                                all_keypoints.push_back(keyps);
                                            }
                                            if(skeletonIn[KeyPointTag::shoulder_center]->isUpdated())
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

    string robot = rf.check("robot", Value("icub")).asString();

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
    lin_est_shoulder=new AWLinEstimator(16,0.01);
    frozen=false;
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
        delete exercises[i];
    }
    for(int i=0; i<processors.size(); i++)
    {
        delete processors[i];
    }
    delete lin_est_shoulder;
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
            if(curr_exercise!=NULL && curr_metric!=NULL &&!frozen)
            {
                if(updated)
                {
                    //update time array
                    time_samples.push_back(Time::now()-tstart);

                    //write on output port
                    Bottle &scopebottleout=scopePort.prepare();
                    scopebottleout.clear();
                    for(int i=0; i<processors.size(); i++)
                    {
                        processors[i]->update(skeletonIn);
                        processors[i]->estimate();
                        Property result=processors[i]->getResult();
                        if(result.check(prop_tag) &&
                                processors[i]->getProcessedMetric()==curr_metric->getParams().find("name").asString())
                        {
                            double res=result.find(prop_tag).asDouble();
                            scopebottleout.addDouble(res);
                        }
                    }
                    scopePort.write();
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

    matvar_t *field;

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

            field = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field,field_vector.data(),MAT_F_GLOBAL);
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

    matvar_t *submatvar;
    for(size_t i=0; i<numMetrics; i++)
    {
        submatvar=writeStructToMat(metrics[i]);
        Mat_VarSetStructFieldByName(met_matvar,fields_metrics[i],0,submatvar);
    }

    Mat_VarSetStructFieldByName(matvar,fields[1],0,met_matvar);
    Mat_VarWrite(matfp,matvar,MAT_COMPRESSION_NONE);
    Mat_VarFree(matvar);

    return true;
}

/********************************************************/
matvar_t * Manager::writeStructToMat(const Metric* m)
{
    matvar_t* submatvar;
    Property params=m->getParams();
    string metric_name=params.find("name").asString();
    string metric_type=params.find("type").asString();
    if(metric_type==MetricType::rom)
    {
        int numFields=8;
        const char *subfields[numFields]={"tag_joint","ref_joint","ref_dir","tag_plane",
                                          "max","min","tstart","tend"};
        size_t dim_struct[2]={1,1};
        submatvar=Mat_VarCreateStruct(metric_name.c_str(),2,dim_struct,subfields,numFields);
        matvar_t *subfield;
        string joint_met=params.find("tag_joint").asString();
        char *joint_c=new char[joint_met.length()+1];
        strcpy(joint_c, joint_met.c_str());
        size_t dims_field_joint[2]={1,joint_met.size()};
        subfield=Mat_VarCreate(NULL,MAT_C_CHAR,MAT_T_UTF8,2,dims_field_joint,joint_c,0);
        Mat_VarSetStructFieldByName(submatvar,subfields[0],0,subfield);
        delete [] joint_c;

        string ref_joint=params.find("ref_joint").asString();
        char *joint_ref=new char[ref_joint.length()+1];
        strcpy(joint_ref,ref_joint.c_str());
        size_t dims_field_refjoint[2]={1,ref_joint.size()};
        subfield=Mat_VarCreate(NULL,MAT_C_CHAR,MAT_T_UTF8,2,dims_field_refjoint,joint_ref,0);
        Mat_VarSetStructFieldByName(submatvar,subfields[1],0,subfield);
        delete [] joint_ref;

        size_t dims_field_dir[2]={1,3};
        Bottle *bRef=params.find("ref_dir").asList();
        Vector ref_met(3,0.0);
        ref_met[0]=bRef->get(0).asDouble();
        ref_met[1]=bRef->get(1).asDouble();
        ref_met[2]=bRef->get(2).asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_dir,ref_met.data(),0);
        Mat_VarSetStructFieldByName(submatvar,subfields[2],0,subfield);

        string tag_plane=params.find("tag_plane").asString();
        char *plane_tag=new char[tag_plane.length()+1];
        strcpy(plane_tag,tag_plane.c_str());
        size_t dims_field_tagplane[2]={1,tag_plane.size()};
        subfield=Mat_VarCreate(NULL,MAT_C_CHAR,MAT_T_UTF8,2,dims_field_tagplane,plane_tag,0);
        Mat_VarSetStructFieldByName(submatvar,subfields[3],0,subfield);
        delete [] plane_tag;

        size_t dims_field_max[2]={1,1};
        double max_val=params.find("max").asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_max,&max_val,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[4],0,subfield);

        size_t dims_field_min[2]={1,1};
        double min_val=params.find("min").asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_min,&min_val,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[5],0,subfield);

        size_t dims_field_tstart[2]={1,1};
        cout<< "started at "<<tstart_session;
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_tstart,&tstart_session,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[6],0,subfield);

        cout<<" ended at "<<tend_session<<endl;
        size_t dims_field_tend[2]={1,1};
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_tend,&tend_session,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[7],0,subfield);
    }
    if(metric_type==MetricType::step)
    {
        int numFields=6;
        const char *subfields[numFields]={"num","den","max","min","tstart","tend"};
        size_t dim_struct[2]={1,1};
        submatvar=Mat_VarCreateStruct(metric_name.c_str(),2,dim_struct,subfields,numFields);
        matvar_t *subfield;
        size_t dims_field_num[2]={1,3};
        Bottle *bNum=params.find("num").asList();
        Vector num(3,0.0);
        num[0]=bNum->get(0).asDouble();
        num[1]=bNum->get(1).asDouble();
        num[2]=bNum->get(2).asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_num,num.data(),0);
        Mat_VarSetStructFieldByName(submatvar,subfields[0],0,subfield);

        size_t dims_field_den[2]={1,3};
        Bottle *bDen=params.find("den").asList();
        Vector den(3,0.0);
        den[0]=bDen->get(0).asDouble();
        den[1]=bDen->get(1).asDouble();
        den[2]=bDen->get(2).asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_den,den.data(),0);
        Mat_VarSetStructFieldByName(submatvar,subfields[1],0,subfield);

        size_t dims_field_max[2]={1,1};
        double max_val=params.find("max").asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_max,&max_val,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[2],0,subfield);

        size_t dims_field_min[2]={1,1};
        double min_val=params.find("min").asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_min,&min_val,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[3],0,subfield);

        size_t dims_field_tstart[2]={1,1};
        cout<<"started at "<<tstart_session;
        subfield = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_tstart,&tstart_session,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[4],0,subfield);

        cout<<" ended at "<<tend_session<<endl;
        size_t dims_field_tend[2]={1,1};
        subfield = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_tend,&tend_session,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[5],0,subfield);
    }
    if(metric_type==MetricType::end_point)
    {
        int numFields=8;
        const char *subfields[numFields]={"tag_joint","ref_dir","tag_plane","max","min","target",
                                          "tstart","tend"};
        size_t dim_struct[2]={1,1};
        submatvar=Mat_VarCreateStruct(metric_name.c_str(),2,dim_struct,subfields,numFields);
        matvar_t *subfield;
        string joint_met=params.find("tag_joint").asString();
        char *joint_c=new char[joint_met.length()+1];
        strcpy(joint_c, joint_met.c_str());
        size_t dims_field_joint[2]={1,joint_met.size()};
        subfield=Mat_VarCreate(NULL,MAT_C_CHAR,MAT_T_UTF8,2,dims_field_joint,joint_c,0);
        Mat_VarSetStructFieldByName(submatvar,subfields[0],0,subfield);
        delete [] joint_c;

        size_t dims_field_dir[2]={1,3};
        Bottle *bRef=params.find("ref_dir").asList();
        Vector ref_met(3,0.0);
        ref_met[0]=bRef->get(0).asDouble();
        ref_met[1]=bRef->get(1).asDouble();
        ref_met[2]=bRef->get(2).asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_dir,ref_met.data(),0);
        Mat_VarSetStructFieldByName(submatvar,subfields[1],0,subfield);

        string tag_plane=params.find("tag_plane").asString();
        char *plane_tag=new char[tag_plane.length()+1];
        strcpy(plane_tag,tag_plane.c_str());
        size_t dims_field_tagplane[2]={1,tag_plane.size()};
        subfield=Mat_VarCreate(NULL,MAT_C_CHAR,MAT_T_UTF8,2,dims_field_tagplane,plane_tag,0);
        Mat_VarSetStructFieldByName(submatvar,subfields[2],0,subfield);
        delete [] plane_tag;

        size_t dims_field_max[2]={1,1};
        double max_val=params.find("max").asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_max,&max_val,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[3],0,subfield);

        size_t dims_field_min[2]={1,1};
        double min_val=params.find("min").asDouble();
        subfield = Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_min,&min_val,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[4],0,subfield);

        size_t dims_field_target[2]={1,3};
        Bottle *bTarget=params.find("target").asList();
        Vector target(3,0.0);
        target[0]=bTarget->get(0).asDouble();
        target[1]=bTarget->get(1).asDouble();
        target[2]=bTarget->get(2).asDouble();
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_target,target.data(),0);
        Mat_VarSetStructFieldByName(submatvar,subfields[5],0,subfield);

        size_t dims_field_tstart[2]={1,1};
        cout<<"started at "<<tstart_session;
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_tstart,&tstart_session,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[6],0,subfield);

        cout<<" ended at "<<tend_session<<endl;
        size_t dims_field_tend[2]={1,1};
        subfield=Mat_VarCreate(NULL,MAT_C_DOUBLE,MAT_T_DOUBLE,2,dims_field_tend,&tend_session,MAT_F_DONT_COPY_DATA);
        Mat_VarSetStructFieldByName(submatvar,subfields[7],0,subfield);
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
