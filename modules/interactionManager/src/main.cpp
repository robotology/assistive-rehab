/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <cmath>
#include <iterator>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>

#include <yarp/os/all.h>
#include <yarp/math/Rand.h>

#include "AssistiveRehab/skeleton.h"
#include "src/interactionManager_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::math;
using namespace assistive_rehab;


/****************************************************************/
class SpeechParam
{
    ostringstream ss;
public:
    SpeechParam(const int d) { ss<<d; }
    SpeechParam(const double g) { ss<<g; }
    SpeechParam(const string &s) { ss<<s; }
    string get() const { return ss.str(); }
};

/****************************************************************/
class MoveThread : public Thread
{
    string file;
    int nrep;
    bool startmoving,motiondone;

public:
    MoveThread() : startmoving(false) { }

    void run() override
    {
        while(!isStopping())
        {
            for(int i=0;i<nrep;i++)
            {
                if(startmoving)
                {
                    motiondone=false;
                    if(system(file.c_str()))
                        yError()<<"Processor not available";
                    motiondone=true;
                }
            }
            if(startmoving)
            {
                stopMoving();
            }
        }
    }

    void stopMoving()
    {
        yInfo()<<"Stop moving";
        startmoving=false;
    }

    bool hasStarted() const
    {
        return startmoving;
    }

    void init(const string & file_, const int nrep_)
    {
        file=file_;
        nrep=nrep_;
        yInfo()<<"Loading"<<file;
        yInfo()<<"Repeat exercise"<<nrep<<"times";
    }

    bool setHomePosition(const string & initialpos)
    {
        yInfo()<<"Home position"<<initialpos;
        if(system(initialpos.c_str()))
            yError()<<"Processor not available";
        return true;
    }

    bool setInitialPosition(const string & initialpos)
    {
        yInfo()<<"Starting position";
        if(system(initialpos.c_str()))
            yError()<<"Processor not available";
        return true;
    }

    void startMoving()
    {
        yInfo()<<"Start moving";
        startmoving=true;
    }

    bool hasStoppedMoving() const
    {
        return (!startmoving && motiondone);
    }
    
};

/****************************************************************/
class Trigger : public BufferedPort<Bottle>
{
    BufferedPort<Bottle> triggerPortOut;

public:

    /********************************************************/
    Trigger()
    {
    }

    /********************************************************/
    ~Trigger()
    {
    };

    /********************************************************/
    bool open(){

        this->useCallback();
        BufferedPort<yarp::os::Bottle >::open("/interactionManager/trigger:i");
        triggerPortOut.open("/interactionManager/trigger:o");
        return true;
    }

    /********************************************************/
    void close()
    {
        BufferedPort<yarp::os::Bottle >::close();
        triggerPortOut.close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle >::interrupt();
        triggerPortOut.close();
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &trigger )
    {
        if(!trigger.isNull())
        {
            Bottle &output=triggerPortOut.prepare();
            output=trigger;
            triggerPortOut.writeStrict();
        }
    }

};


/****************************************************************/
class Interaction : public RFModule, public interactionManager_IDL
{
    const int ok=Vocab::encode("ok");
    const int fail=Vocab::encode("fail");

    enum class State { stopped, idle, seek, follow, engaged, move, show, imitated } state;
    double period;
    string tag;
    double t0;
    int reinforce_engage_cnt;
    bool use_robot_template;
    string robot_skeleton_name;

    MoveThread *movethr;
    string move_file,motion_type;
    vector<double> engage_distance,engage_azimuth;
    bool mirror_exercise;
    vector<double> panel_size,panel_pose_left,panel_pose_right;
    vector<int> panel_color;
    string panelid;
    int nrep_show,nrep_perform;
    bool virtual_mode,engage_with_hand,wait_for_imitation;
    bool first_run;

    unordered_map<string,vector<string>> history;
    unordered_map<string,string> speak_map;
    vector<double> assess_values;
    vector<string> parttomove;
    bool occluded;
    string partrob;
    string motion_type_robot,script_starting,script_move,script_home;
    bool imitate,observe;
    string partspeech;

    Mutex mutex;

    RpcClient attentionPort;
    RpcClient analyzerPort;
    RpcClient worldGazeboPort;
    RpcClient robotSkeletonPort;
    BufferedPort<Bottle> synthetizerPort;
    BufferedPort<Bottle> speechStreamPort;
    RpcClient speechRpcPort;
    RpcServer cmdPort;
    bool interrupting;
    Trigger *trigger;

    /****************************************************************/
    bool start_with_hand() override
    {
        LockGuard lg(mutex);
        engage_with_hand=true;
        wait_for_imitation=false;
        return disengage();
    }

    /****************************************************************/
    bool start_observation() override
    {
        LockGuard lg(mutex);
        engage_with_hand=false;
        wait_for_imitation=true;
        observe=true;
        return disengage();
    }

    /****************************************************************/
    bool start_imitation() override
    {
        LockGuard lg(mutex);
        imitate=true;
        return true;
    }

    /****************************************************************/
    bool start_occlusion() override
    {
        LockGuard lg(mutex);
        occluded=true;
        if(virtual_mode)
        {
            //add box in gazebo
            occluded=false;
            Bottle cmd,rep;
            cmd.addString("makeBox");
            cmd.addDouble(panel_size[0]); //width
            cmd.addDouble(panel_size[1]); //height
            cmd.addDouble(panel_size[2]); //thickness
            if(partrob=="left")
            {
                cmd.addDouble(panel_pose_left[0]); //pose.x
                cmd.addDouble(panel_pose_left[1]); //pose.y
                cmd.addDouble(panel_pose_left[2]); //pose.z
                cmd.addDouble(panel_pose_left[3]); //pose.roll
                cmd.addDouble(panel_pose_left[4]); //pose.pitch
                cmd.addDouble(panel_pose_left[5]); //pose.yaw
            }
            else
            {
                cmd.addDouble(panel_pose_right[0]); //pose.x
                cmd.addDouble(panel_pose_right[1]); //pose.y
                cmd.addDouble(panel_pose_right[2]); //pose.z
                cmd.addDouble(panel_pose_right[3]); //pose.roll
                cmd.addDouble(panel_pose_right[4]); //pose.pitch
                cmd.addDouble(panel_pose_right[5]); //pose.yaw
            }
            cmd.addInt(panel_color[0]); //color.r
            cmd.addInt(panel_color[1]); //color.g
            cmd.addInt(panel_color[2]); //color.b
            if(worldGazeboPort.write(cmd,rep))
            {
                if (!rep.get(0).asString().empty())
                {
                    //time for the object to be created
                    Time::delay(1.0);
                    panelid=rep.get(0).asString();
                    cmd.clear();
                    rep.clear();
                    cmd.addString("enableGravity");
                    cmd.addString(panelid);
                    cmd.addInt(1);
                    worldGazeboPort.write(cmd,rep);
                    if(rep.get(0).asVocab()==ok)
                    {
                        occluded=true;
                    }
                }
            }
        }

        return occluded;
    }

    /****************************************************************/
    bool stop() override
    {
        LockGuard lg(mutex);
        bool ret=false;

        Bottle cmd,rep;
        cmd.addString("stop");
        if (attentionPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==ok)
            {
                if (state==State::move)
                {
                    Bottle cmd,rep;
                    cmd.addString("stop");
                    analyzerPort.write(cmd,rep);
                }
                ret=true;
            }
        }

        if(imitate)
        {
            imitate=false;
        }

        if(occluded)
        {
            if(virtual_mode)
            {
                ret=false;
                cmd.clear();
                rep.clear();
                cmd.addString("deleteObject");
                cmd.addString(panelid);
                worldGazeboPort.write(cmd,rep);
                if(rep.get(0).asVocab()==ok)
                {
                    occluded=false;
                    ret=true;
                }
            }
            else
            {
                occluded=false;
                ret=true;
            }
        }

        if(movethr->hasStarted())
        {
            movethr->stopMoving();
        }
        while(!movethr->hasStoppedMoving())
        {
            Time::yield();
        }
        ret=true;
        movethr->setHomePosition(script_home);

        state=State::stopped;
        return ret;
    }

    /****************************************************************/
    bool speak(const string &key, const bool wait,
               const vector<SpeechParam> &p=vector<SpeechParam>())
    {
        auto it=speak_map.find(key);
        string value=(it!=end(speak_map)?it->second:speak_map["ouch"]);

        string value_ext;
        if (!p.empty())
        {
            for (size_t i=0;;)
            {
                size_t pos=value.find("%");
                value_ext+=value.substr(0,pos);
                if (i<p.size())
                {
                    value_ext+=p[i++].get();
                }
                if (pos==string::npos)
                {
                    break;
                }
                value.erase(0,pos+1);
            }
        }
        else
        {
            value_ext=value;
        }

        Bottle &payload=speechStreamPort.prepare();
        payload.clear();
        payload.addString(value_ext);
        speechStreamPort.writeStrict();

        while (wait && !interrupting && (speechRpcPort.getOutputCount()>0))
        {
            Time::delay(getPeriod());
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("stat"));
            if (speechRpcPort.write(cmd,rep))
            {
                if (rep.get(0).asString()=="quiet")
                {
                    break;
                }
            }
        }
        return (it!=end(speak_map));
    }

    /****************************************************************/
    bool attach(RpcServer &source) override
    {
        return yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool load_speak(const string &context, const string &speak_file)
    {
        ResourceFinder rf_speak;
        rf_speak.setDefaultContext(context);
        rf_speak.setDefaultConfigFile(speak_file.c_str());
        rf_speak.configure(0,nullptr);

        Bottle &bGroup=rf_speak.findGroup("general");
        if (bGroup.isNull())
        {
            yError()<<"Unable to find group \"general\"";
            return false;
        }
        if (!bGroup.check("num-sections") || !bGroup.check("part"))
        {
            yError()<<"Unable to find key \"num-sections\" or \"part\"";
            return false;
        }
        int num_sections=bGroup.find("num-sections").asInt();
        parttomove.resize(2);
        if(Bottle *bPart=bGroup.find("part").asList())
        {
            parttomove[0]=bPart->get(0).asString();
            parttomove[1]=bPart->get(1).asString();
        }
        for (int i=0; i<num_sections; i++)
        {
            ostringstream section;
            section<<"section-"<<i;
            Bottle &bSection=rf_speak.findGroup(section.str());
            if (bSection.isNull())
            {
                string msg="Unable to find section";
                msg+="\""+section.str()+"\"";
                yError()<<msg;
                return false;
            }
            if (!bSection.check("key") || !bSection.check("value"))
            {
                yError()<<"Unable to find key \"key\" and/or \"value\"";
                return false;
            }
            string key=bSection.find("key").asString();
            string value=bSection.find("value").asString();
            speak_map[key]=value;
        }

        return true;
    }

    /****************************************************************/
    bool disengage()
    {
        bool ret=false;
        state=State::idle;

        if(imitate)
        {
            imitate=false;
            //a new interaction can only start when next start_observation is given
            state=State::stopped;
        }

        Bottle cmd,rep;
        cmd.addString("stop");
        if (attentionPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==ok)
            {
                cmd.clear();
                cmd.addString("set_auto");
                if (attentionPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==ok)
                    {
                        if(virtual_mode)
                        {
                            cmd.clear();
                            cmd.addString("set_virtual");
                            if (attentionPort.write(cmd,rep))
                            {
                                if (rep.get(0).asVocab()==ok)
                                {
                                    ret=true;
                                }
                            }
                        }
                        else
                        {
                            ret=true;
                        }
                    }
                }
            }
        }

        if(occluded)
        {
            ret=false;
            Bottle cmd,rep;
            cmd.addString("deleteObject");
            cmd.addString(panelid);
            worldGazeboPort.write(cmd,rep);
            if(rep.get(0).asVocab()==ok)
            {
                ret=true;
                occluded=false;
            }
        }

        if(movethr->hasStarted())
        {
            ret=false;
            movethr->stopMoving();
        }
        while(!movethr->hasStoppedMoving())
        {
            Time::yield();
        }
        ret=true;
        movethr->setHomePosition(script_home);

        t0=Time::now();
        return ret;
    }

    /****************************************************************/
    string select_randomly(const Bottle &s)
    {
        string selected;
        double p=Rand::scalar(0,1);
        auto it1=history.find(tag);
        if (it1==end(history))
        {
            selected=s.get((int)floor(p*s.size())).asString();
        }
        else
        {
            vector<string> s1,diff;
            for (int i=0; i<s.size(); i++)
            {
                s1.push_back(s.get(i).asString());
            }
            sort(begin(s1),end(s1));

            auto &s2=it1->second;
            sort(begin(s2),end(s2));

            set_difference(begin(s1),end(s1),begin(s2),end(s2),inserter(diff,end(diff)));
            if (diff.empty())
            {
                diff=s1;
                s2.clear();
            }
            auto it2=diff.begin();
            advance(it2,(int)floor(p*diff.size()));
            selected=*it2;
        }
        return selected;
    }

    /****************************************************************/
    bool assess(const string &phase)
    {
        if (assess_values.empty() ||
            ((phase!="intermediate") && (phase!="final")))
        {
            speak("ouch",true);
            return false;
        }

        double mean=0.0;
        for (auto &v:assess_values)
        {
            mean+=v;
        }
        mean/=assess_values.size();

        string grade;
        if (mean<0.4)
        {
            grade="low";
        }
        else if (mean<0.7)
        {
            grade="medium";
        }
        else
        {
            grade="high";
        }
        yInfo()<<"Score"<<mean<<grade;
        speak("assess-"+phase+"-"+grade,true);
        return true;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        period=rf.check("period",Value(0.1)).asDouble();
        string speak_file=rf.check("speak-file",Value("speak-it")).asString();
        move_file=rf.check("move-file",Value("run-movements.sh")).asString();

        engage_distance=vector<double>{2.0,4.0};
        if (Bottle *p=rf.find("engage-distance").asList())
        {
            if (p->size()>=2)
            {
                engage_distance[0]=p->get(0).asDouble();
                engage_distance[1]=p->get(1).asDouble();
            }
        }

        engage_azimuth=vector<double>{-30.0,30.0};
        if (Bottle *p=rf.find("engage-azimuth").asList())
        {
            if (p->size()>=2)
            {
                engage_azimuth[0]=p->get(0).asDouble();
                engage_azimuth[1]=p->get(1).asDouble();
            }
        }

        mirror_exercise=rf.check("mirror-exercise",Value(true)).asBool();

        panel_size.resize(3,0.0);
        if (Bottle *p=rf.find("panel-size").asList())
        {
            panel_size[0]=p->get(0).asDouble();
            panel_size[1]=p->get(1).asDouble();
            panel_size[2]=p->get(2).asDouble();
        }

        panel_pose_right.resize(6,0.0);
        if (Bottle *p=rf.find("panel-pose-right").asList())
        {
            panel_pose_right[0]=p->get(0).asDouble();
            panel_pose_right[1]=p->get(1).asDouble();
            panel_pose_right[2]=p->get(2).asDouble();
            panel_pose_right[3]=p->get(3).asDouble();
            panel_pose_right[4]=p->get(4).asDouble();
            panel_pose_right[5]=p->get(5).asDouble();
        }

        panel_pose_left.resize(6,0.0);
        if (Bottle *p=rf.find("panel-pose-left").asList())
        {
            panel_pose_left[0]=p->get(0).asDouble();
            panel_pose_left[1]=p->get(1).asDouble();
            panel_pose_left[2]=p->get(2).asDouble();
            panel_pose_left[3]=p->get(3).asDouble();
            panel_pose_left[4]=p->get(4).asDouble();
            panel_pose_left[5]=p->get(5).asDouble();
        }

        panel_color.resize(3,255);
        if (Bottle *p=rf.find("panel-color").asList())
        {
            panel_color[0]=p->get(0).asInt();
            panel_color[1]=p->get(1).asInt();
            panel_color[2]=p->get(2).asInt();
        }

        nrep_show=rf.check("nrep-show",Value(2)).asInt();
        nrep_perform=rf.check("nrep-perform",Value(7)).asInt();
        virtual_mode=rf.check("virtual-mode",Value(false)).asBool();
        engage_with_hand=rf.check("engage-with-hand",Value(true)).asBool();
        wait_for_imitation=rf.check("wait-for-imitation",Value(false)).asBool();
        use_robot_template=rf.check("use-robot-template",Value(true)).asBool();
        robot_skeleton_name=rf.check("robot-skeleton-name",Value("robot")).asString();

        if (!load_speak(rf.getContext(),speak_file))
        {
            string msg="Unable to locate file";
            msg+="\""+speak_file+"\"";
            yError()<<msg;
            return false;
        }

        attentionPort.open("/interactionManager/attention:rpc");
        worldGazeboPort.open("/interactionManager/gazebo:rpc");
        analyzerPort.open("/interactionManager/analyzer:rpc");
        synthetizerPort.open("/interactionManager/synthetizer:i");
        robotSkeletonPort.open("/interactionManager/robotskeleton:rpc");
        speechStreamPort.open("/interactionManager/speech:o");
        speechRpcPort.open("/interactionManager/speech:rpc");
        cmdPort.open("/interactionManager/cmd:rpc");

        trigger=new Trigger();
        trigger->open();

        attach(cmdPort);

        Rand::init();
        state=State::idle;
        interrupting=false;
        occluded=false;
        imitate=false;
        observe=false;
        t0=Time::now();
        first_run=true;

        movethr=new MoveThread();
        if(!movethr->start())
        {
            yError()<<"Move thread not started!";
            delete movethr;
            return false;
        }

        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return period;
    }

    /****************************************************************/
    bool updateModule() override
    {
        LockGuard lg(mutex);
        if ((attentionPort.getOutputCount()==0) || (analyzerPort.getOutputCount()==0) ||
                (speechStreamPort.getOutputCount()==0) || (speechRpcPort.getOutputCount()==0) ||
                (robotSkeletonPort.getOutputCount()==0))
        {
            yInfo()<<"not connected";
            return true;
        }

        if(first_run)
        {
            Bottle cmd,rep;
            cmd.addString("set_robot_skeleton");
            cmd.addString(robot_skeleton_name);
            if(robotSkeletonPort.write(cmd,rep))
            {
                yInfo()<<"Setting robot skeleton name as"<<robot_skeleton_name;
                rep.clear();
                if (attentionPort.write(cmd,rep))
                {
                    yInfo()<<"Cannot follow"<<robot_skeleton_name;
                }

                if(use_robot_template)
                {
                    cmd.clear();
                    rep.clear();
                    cmd.addString("mirrorTemplate");
                    cmd.addInt(mirror_exercise);
                    if (analyzerPort.write(cmd,rep))
                    {
                        if(mirror_exercise)
                        {
                            yInfo()<<"Mirroring robot template";
                        }
                        else
                        {
                            yInfo()<<"Not mirroring robot template";
                        }
                    }
                }
                else
                {
                    cmd.clear();
                    rep.clear();
                    cmd.addString("set_visibility");
                    cmd.addInt(0);
                    if(robotSkeletonPort.write(cmd,rep))
                    {
                        yInfo()<<"Turn off robot template visibility";
                    }
                }
            }
            first_run=false;
        }

        string follow_tag("");
        bool is_follow_tag_ahead=false;
        {
            Bottle cmd,rep;
            cmd.addString("is_following");
            if (attentionPort.write(cmd,rep))
            {
                follow_tag=rep.get(0).asString();
                double x=rep.get(1).asDouble();
                double y=rep.get(2).asDouble();
                double z=rep.get(3).asDouble();

                double r=sqrt(x*x+y*y+z*z);
                double azi=(180.0/M_PI)*atan2(y,x);
                is_follow_tag_ahead=(r>engage_distance[0]) && (r<engage_distance[1]) &&
                                    (azi>engage_azimuth[0]) && (azi<engage_azimuth[1]);
            }
        }
        if (state>=State::follow)
        {
            if (follow_tag!=tag)
            {
                Bottle cmd,rep;
                cmd.addString("stop");
                analyzerPort.write(cmd,rep);
                speak("disengaged",true);
                disengage();
                return true;
            }
        }

        if (state==State::idle)
        {
            if (Time::now()-t0>3.0)
            {
                state=State::seek;
            }
        }

        if (state==State::seek)
        {            
            if (!follow_tag.empty() && is_follow_tag_ahead)
            {                
                tag=follow_tag;
                Bottle cmd,rep;
                cmd.addString("look");
                cmd.addString(tag);
                cmd.addString(KeyPointTag::shoulder_center);

                if (attentionPort.write(cmd,rep))
                {
                    vector<SpeechParam> p;
                    p.push_back(SpeechParam(tag[0]!='#'?tag:string("")));
                    if(engage_with_hand)
                    {
                        speak(history.find(tag)==end(history)?
                              "invite-start":"invite-cont",true,p);
                        speak("engage",true);
                    }
                    state=State::follow;
                    reinforce_engage_cnt=0;
                    t0=Time::now();
                }
            }
        }

        if (state==State::follow)
        {
            if(engage_with_hand)
            {
                Bottle cmd,rep;
                cmd.addString("is_with_raised_hand");
                cmd.addString(tag);

                if (attentionPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==ok)
                    {
                        speak("accepted",true);
                        state=State::engaged;
                    }
                    else if (Time::now()-t0>10.0)
                    {
                        if (++reinforce_engage_cnt<=1)
                        {
                            speak("reinforce-engage",true);
                            t0=Time::now();
                        }
                        else
                        {
                            speak("disengaged",true);
                            disengage();
                        }
                    }
                }
            }
            else
            {
                state=State::engaged;
            }
        }

        if (state==State::engaged)
        {
            Bottle cmd,rep;
            cmd.addString("listExercises");
            if (analyzerPort.write(cmd,rep))
            {
                Bottle &exercises=*rep.get(0).asList();
                if (exercises.size()>0)
                {
                    string exercise=select_randomly(exercises);
                    yInfo()<<"Selected exercise:"<<exercise;

                    cmd.clear();
                    rep.clear();
                    cmd.addString("loadExercise");
                    cmd.addString(exercise);
                    if (analyzerPort.write(cmd,rep))
                    {
                        bool ack=rep.get(0).asBool();
                        if (ack)
                        {
                            cmd.clear();
                            cmd.addString("listMetrics");
                            if (analyzerPort.write(cmd,rep))
                            {
                                Bottle &metrics=*rep.get(0).asList();
                                if (metrics.size()>0)
                                {
                                    string metric=select_randomly(metrics);
                                    yInfo()<<"Selected metric:"<<metric;

                                    cmd.clear();
                                    rep.clear();
                                    cmd.addString("selectMetric");
                                    cmd.addString(metric);
                                    if (analyzerPort.write(cmd,rep))
                                    {
                                        ack=rep.get(0).asBool();
                                        if(ack)
                                        {
                                            cmd.clear();
                                            rep.clear();
                                            cmd.addString("listMetricProps");
                                            if (analyzerPort.write(cmd,rep))
                                            {
                                                Bottle &props=*rep.get(0).asList();
                                                if (props.size()>0)
                                                {
                                                    string prop=select_randomly(props);
                                                    yInfo()<<"Selected prop:"<<prop;

                                                    cmd.clear();
                                                    rep.clear();
                                                    cmd.addString("selectMetricProp");
                                                    cmd.addString(prop);
                                                    if (analyzerPort.write(cmd,rep))
                                                    {
                                                        ack=rep.get(0).asBool();
                                                        if(ack)
                                                        {
                                                            cmd.clear();
                                                            rep.clear();
                                                            cmd.addString("getExercise");
                                                            if(analyzerPort.write(cmd,rep))
                                                            {
                                                                motion_type=rep.get(0).asString();
                                                                string template_tag;
                                                                if(use_robot_template)
                                                                {
                                                                    template_tag=robot_skeleton_name;
                                                                }
                                                                else
                                                                {
                                                                    template_tag=motion_type;
                                                                }

                                                                cmd.clear();
                                                                rep.clear();
                                                                cmd.addString("setTemplateTag");
                                                                cmd.addString(template_tag);
                                                                if(analyzerPort.write(cmd,rep))
                                                                {
                                                                    size_t found=exercise.find_last_of("_");
                                                                    string part=exercise.substr(found+1,exercise.size());
                                                                    if(mirror_exercise)
                                                                    {
                                                                        if(part=="left")
                                                                        {
                                                                            partrob="right";
                                                                            partspeech=parttomove[1];
                                                                        }
                                                                        if(part=="right")
                                                                        {
                                                                            partrob="left";
                                                                            partspeech=parttomove[0];
                                                                        }
                                                                    }
                                                                    else
                                                                    {
                                                                        if(part=="left")
                                                                        {
                                                                            partrob="left";
                                                                            partspeech=parttomove[1];
                                                                        }
                                                                        if(part=="right")
                                                                        {
                                                                            partrob="right";
                                                                            partspeech=parttomove[0];
                                                                        }
                                                                    }
                                                                    cmd.clear();
                                                                    rep.clear();
                                                                    cmd.addString("setPart");
                                                                    cmd.addString(part);
                                                                    yInfo()<<"Analyzing"<<part<<"arm";
                                                                    if (analyzerPort.write(cmd,rep))
                                                                    {
                                                                        motion_type_robot=exercise.substr(0,found)+"_"+partrob;
                                                                        script_starting=move_file+" "+"startingpos_"+motion_type_robot;
                                                                        script_move=move_file+" "+motion_type_robot;
                                                                        script_home=move_file+" "+"home_"+motion_type_robot;

                                                                        cmd.clear();
                                                                        rep.clear();
                                                                        cmd.addString("selectSkel");
                                                                        cmd.addString(tag);
                                                                        yInfo()<<"Selecting skeleton"<<tag;
                                                                        if (analyzerPort.write(cmd,rep))
                                                                        {
                                                                            if (rep.get(0).asVocab()==ok)
                                                                            {
                                                                                //imitation and observation phases are not separated
                                                                                //the robot shows the exercise and then the user imitates
                                                                                //we don't wait for the imitation to be started externally
                                                                                if(!wait_for_imitation)
                                                                                {
                                                                                    if (history.find(tag)==end(history))
                                                                                    {
                                                                                        speak("explain",true);
                                                                                    }
                                                                                    else
                                                                                    {
                                                                                        vector<SpeechParam> p;
                                                                                        p.push_back(SpeechParam(tag[0]!='#'?(tag+","):string("")));
                                                                                        speak("in-the-know",true,p);
                                                                                    }

                                                                                    vector<SpeechParam> p;
                                                                                    p.push_back(SpeechParam(partspeech));
                                                                                    speak("show",true,p);
                                                                                    movethr->setInitialPosition(script_starting);
                                                                                    movethr->init(script_move,nrep_show);
                                                                                    movethr->startMoving();
                                                                                    while(!movethr->hasStoppedMoving())
                                                                                    {
                                                                                        //wait until it finishes
                                                                                        Time::yield();
                                                                                    }

                                                                                    cmd.clear();
                                                                                    rep.clear();
                                                                                    cmd.addString("is_following");
                                                                                    if (attentionPort.write(cmd,rep))
                                                                                    {
                                                                                        follow_tag=rep.get(0).asString();
                                                                                        if (follow_tag!=tag)
                                                                                        {
                                                                                            speak("disengaged",true);
                                                                                            disengage();
                                                                                            return true;
                                                                                        }
                                                                                    }

                                                                                    movethr->setHomePosition(script_home);
                                                                                    movethr->setInitialPosition(script_starting);
                                                                                    Time::delay(3.0);
                                                                                    speak("start",true);
                                                                                    history[tag].push_back(exercise);
                                                                                    movethr->init(script_move,nrep_perform);
                                                                                    movethr->startMoving();
                                                                                    Time::delay(1.0);

                                                                                    cmd.clear();
                                                                                    cmd.addString("start");
                                                                                    cmd.addInt(use_robot_template);
                                                                                    if (analyzerPort.write(cmd,rep))
                                                                                    {
                                                                                        if (rep.get(0).asVocab()==ok)
                                                                                        {
                                                                                            state=State::move;

                                                                                            assess_values.clear();
                                                                                            t0=Time::now();
                                                                                        }
                                                                                    }
                                                                                }
                                                                                else
                                                                                {
                                                                                    //we have two phases: observation and imitation
                                                                                    //observation starts when the command start_observation is given
                                                                                    //imitation starts when the command start_imitation is given
                                                                                    state=State::show;
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
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if (state==State::show)
        {
            if(observe)
            {
                speak("welcome",true);
                vector<SpeechParam> p;
                p.push_back(SpeechParam(partspeech));
                movethr->setInitialPosition(script_starting);
                speak("show",true,p);
                movethr->init(script_move,nrep_show);
                movethr->startMoving();

                state=State::imitated;
                observe=false;
            }
        }

        if (state==State::imitated)
        {
            if(imitate)
            {
                speak("start",true);
                movethr->init(script_move,nrep_perform);
                movethr->startMoving();
                Time::delay(0.5);

                Bottle cmd,rep;
                cmd.addString("start");
                cmd.addInt(use_robot_template);
                if (analyzerPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==ok)
                    {
                        state=State::move;

                        assess_values.clear();
                        t0=Time::now();
                    }
                }
            }
        }

        if(state==State::move)
        {
            if(!movethr->hasStoppedMoving())
            {
                if(occluded)
                {
                    Bottle cmd,rep;
                    cmd.addString("stopFeedback");
                    if (analyzerPort.write(cmd,rep))
                    {
                        if (rep.get(0).asVocab()==ok)
                        {
                            yInfo()<<"Stopping feedback";
                        }
                    }
                }
                else
                {
                    if(Bottle *score = synthetizerPort.read(false))
                        assess_values.push_back(score->get(0).asDouble());
                }
            }
            else
            {
                yInfo()<<"Stopping";
                movethr->setHomePosition(script_home);
                Time::delay(1.0);
                Bottle cmd,rep;
                cmd.addString("stop");
                analyzerPort.write(cmd,rep);

                speak("end",true);
                if(engage_with_hand)
                {
                    assess("final");
                }

                speak("greetings",true);
                disengage();
            }
        }

        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        interrupting=true;
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        movethr->stop();
        delete movethr;

        trigger->interrupt();
        trigger->close();
        delete trigger;

        attentionPort.close();
        worldGazeboPort.close();
        analyzerPort.close();
        synthetizerPort.close();
        robotSkeletonPort.close();
        speechStreamPort.close();
        speechRpcPort.close();
        cmdPort.close();
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
    rf.setDefaultContext("interactionManager");
    rf.setDefaultConfigFile("config-it.ini");
    rf.configure(argc,argv);

    Interaction interaction;
    return interaction.runModule(rf);
}


