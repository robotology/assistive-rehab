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
    bool startmoving;

public:
    MoveThread() : startmoving(false) { }

    void run() override
    {
        while(!isStopping())
        {
            if(startmoving)
            {
                yInfo()<<"Loading"<<file;
                if(system(file.c_str()))
                    yError()<<"Processor not available";
                stopMoving();
            }
        }
    }

    void stopMoving()
    {
        yInfo()<<"Stop moving";
        startmoving=false;
    }

    bool isMoving() const
    {
        return startmoving;
    }

    void setFile(const string & file_)
    {
        file=file_;
    }

    void startMoving()
    {
        yInfo()<<"Start moving";
        startmoving=true;
    }
    
};


/****************************************************************/
class Interaction : public RFModule, public interactionManager_IDL
{
    const int ok=Vocab::encode("ok");
    const int fail=Vocab::encode("fail");

    enum class State { stopped, idle, seek, follow, engaged, move } state;
    double period;
    string tag;
    double t0;
    int reinforce_engage_cnt;

    MoveThread *movethr;
    string move_file,motion_type;
    vector<double> engage_distance,engage_azimuth;
    bool mirror_exercise;
    vector<double> panel_size,panel_pose;
    vector<int> panel_color;
    string panelid;

    unordered_map<string,vector<string>> history;
    unordered_map<string,string> speak_map;
    vector<double> assess_values;
    vector<string> parttomove;
    bool occluded,keepmoving;

    Mutex mutex;

    RpcClient attentionPort;
    RpcClient analyzerPort;
    RpcClient worldGazeboPort;
    BufferedPort<Bottle> synthetizerPort;
    BufferedPort<Bottle> speechStreamPort;
    RpcClient speechRpcPort;
    RpcServer cmdPort;
    bool interrupting;

    /****************************************************************/
    bool start() override
    {
        LockGuard lg(mutex);
        return disengage();
    }

    /****************************************************************/
    bool start_occlusion() override
    {
        LockGuard lg(mutex);

        //add box in gazebo
        Bottle cmd,rep;
        cmd.addString("makeBox");
        cmd.addDouble(panel_size[0]); //width
        cmd.addDouble(panel_size[1]); //height
        cmd.addDouble(panel_size[2]); //thickness
        cmd.addDouble(panel_pose[0]); //pose.x
        cmd.addDouble(panel_pose[1]); //pose.y
        cmd.addDouble(panel_pose[2]); //pose.z
        cmd.addDouble(panel_pose[3]); //pose.roll
        cmd.addDouble(panel_pose[4]); //pose.pitch
        cmd.addDouble(panel_pose[5]); //pose.yaw
        cmd.addInt(panel_color[0]); //color.r
        cmd.addInt(panel_color[1]); //color.g
        cmd.addInt(panel_color[2]); //color.b
        if(worldGazeboPort.write(cmd,rep))
        {
            if (!rep.get(0).asString().empty())
            {
                //time for the object to be created
                Time::delay(0.2);
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
                        ret=true;
                    }
                }
            }
        }

        if(occluded)
        {
            ret=false;
            cmd.clear();
            rep.clear();
            cmd.addString("deleteObject");
            cmd.addString(panelid);
            if(rep.get(0).asVocab()==ok)
            {
                ret=true;
            }
        }

        t0=Time::now();
        return ret;
    }

    /****************************************************************/
    string select_metric(const Bottle &metrics)
    {
        string metric;
        double p=Rand::scalar(0,1);
        auto it1=history.find(tag);
        if (it1==end(history))
        {
            metric=metrics.get((int)floor(p*metrics.size())).asString();
        }
        else
        {
            vector<string> s1,diff;
            for (int i=0; i<metrics.size(); i++)
            {
                s1.push_back(metrics.get(i).asString());
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
            metric=*it2;
        }
        return metric;
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

        panel_pose.resize(6,0.0);
        if (Bottle *p=rf.find("panel-pose").asList())
        {
            panel_pose[0]=p->get(0).asDouble();
            panel_pose[1]=p->get(1).asDouble();
            panel_pose[2]=p->get(2).asDouble();
            panel_pose[3]=p->get(3).asDouble();
            panel_pose[4]=p->get(4).asDouble();
            panel_pose[5]=p->get(5).asDouble();
        }

        panel_color.resize(3,255);
        if (Bottle *p=rf.find("panel-color").asList())
        {
            panel_color[0]=p->get(0).asInt();
            panel_color[1]=p->get(1).asInt();
            panel_color[2]=p->get(2).asInt();
        }

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
        speechStreamPort.open("/interactionManager/speech:o");
        speechRpcPort.open("/interactionManager/speech:rpc");
        cmdPort.open("/interactionManager/cmd:rpc");
        attach(cmdPort);

        Rand::init();
        state=State::idle;
        interrupting=false;
        occluded=false;
        keepmoving=false;
        t0=Time::now();

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
                (speechStreamPort.getOutputCount()==0) || (speechRpcPort.getOutputCount()==0))
        {
            yInfo()<<"not connected";
            return true;
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
                if (state==State::move)
                {
                    Bottle cmd,rep;
                    cmd.addString("stop");
                    analyzerPort.write(cmd,rep);
                }
                speak("disengaged",true);
                disengage();
                return true;
            }
        }

        if (state==State::idle)
        {
            if (Time::now()-t0>10.0)
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
                    speak(history.find(tag)==end(history)?
                          "invite-start":"invite-cont",true,p);
                    speak("engage",true);
                    state=State::follow;
                    reinforce_engage_cnt=0;
                    t0=Time::now();
                }
            }
        }

        if (state==State::follow)
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

        if (state==State::engaged)
        {
            Bottle cmd,rep;
            cmd.addString("listMetrics");
            if (analyzerPort.write(cmd,rep))
            {
                Bottle &metrics=*rep.get(0).asList();
                if (metrics.size()>0)
                {
                    string metric=select_metric(metrics);
                    yInfo()<<"Selected metric:"<<metric;

                    cmd.clear();
                    cmd.addString("loadMetric");
                    cmd.addString(metric);
                    if (analyzerPort.write(cmd,rep))
                    {
                        bool ack=rep.get(0).asBool();
                        if (ack)
                        {
                            cmd.clear();
                            cmd.addString("getMotionType");
                            if(analyzerPort.write(cmd,rep))
                            {
                                motion_type=rep.get(0).asString();
                                size_t found=motion_type.find_last_of("_");
                                string part=motion_type.substr(found+1,motion_type.size());
                                string partrob,partspeech;
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

                                if (analyzerPort.write(cmd,rep))
                                {
                                    if (rep.get(0).asVocab()==ok)
                                    {
                                        string motion_type_robot=motion_type.substr(0,found)+"_"+partrob;
                                        string script_show=move_file+" "+"show_"+motion_type_robot;
                                        string script_perform=move_file+" "+"perform_"+motion_type_robot;

                                        cmd.clear();
                                        cmd.addString("selectSkel");
                                        cmd.addString(tag);
                                        yInfo()<<"Selecting skeleton"<<tag;
                                        if (analyzerPort.write(cmd,rep))
                                        {
                                            if (rep.get(0).asVocab()==ok)
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
                                                movethr->setFile(script_show);
                                                movethr->startMoving();

                                                while(movethr->isMoving())
                                                {
                                                    //wait until it finishes
                                                    Time::yield();
                                                }

                                                Time::delay(3.0);
                                                speak("start",true);
                                                history[tag].push_back(metric);
                                                movethr->setFile(script_perform);
                                                movethr->startMoving();
                                                Time::delay(1.0);

                                                cmd.clear();
                                                cmd.addString("start");
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

                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (state!=State::move)
            {
                Bottle cmd,rep;
                cmd.addString("stop");
                analyzerPort.write(cmd,rep);

                speak("ouch",true);
                disengage();
            }
        }

        if(state==State::move)
        {
            if(movethr->isMoving())
            {
                if(occluded && !keepmoving)
                {
                    Bottle cmd,rep;
                    cmd.addString("stop_feedback");
                    if (analyzerPort.write(cmd,rep))
                    {
                        if (rep.get(0).asVocab()==ok)
                        {
                            yInfo()<<"Stopping feedback";
                            keepmoving=true;
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
                Time::delay(1.0);
                Bottle cmd,rep;
                cmd.addString("stop");
                analyzerPort.write(cmd,rep);

                speak("end",true);
                if(!occluded)
                    assess("final");
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
        attentionPort.close();
        worldGazeboPort.close();
        analyzerPort.close();
        synthetizerPort.close();
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


