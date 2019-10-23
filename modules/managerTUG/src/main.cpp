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

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/math/Math.h>

#include <mutex>
#include <cmath>

#include <AssistiveRehab/skeleton.h>

#include "src/managerTUG_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class SpeechInterpApi : public BufferedPort<Bottle>
{
    string module_name;
    BufferedPort<Bottle> speechPortOut;
    unordered_map<string,string> speak_map;
    double t;
    string answer;

public:

    /********************************************************/
    SpeechInterpApi(const string &module_name, const unordered_map<string,string> &speak_map)
    {
        this->module_name=module_name;
        this->speak_map=speak_map;
    }

    /********************************************************/
    ~SpeechInterpApi()
    {
    };

    /********************************************************/
    bool open()
    {
        this->useCallback();
        BufferedPort<yarp::os::Bottle >::open("/"+module_name+"/speech-interp:i");
        speechPortOut.open("/"+module_name+"/speech-interp:o");
        return true;
    }

    /********************************************************/
    void close()
    {
        BufferedPort<yarp::os::Bottle >::close();
        speechPortOut.close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle >::interrupt();
        speechPortOut.interrupt();
    }

    /********************************************************/
    void setTime(const double &t)
    {
        this->t=t;
    }

    /********************************************************/
    string getAnswer() const
    {
        return answer;
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &speech_interpretation )
    {
        if(!speech_interpretation.isNull())
        {
            string keyword=speech_interpretation.get(0).asString();
            answer.clear();
            if(keyword=="speed" || keyword=="aid" || keyword=="repetition"
                    || keyword=="unclear" || keyword=="not-known")
            {
                answer=speak_map[keyword];
            }
            else if(keyword=="feedback")
            {
                if(t<=10.0)
                {
                    answer=speak_map[keyword+"-high"];
                }
                else if(t<=20.0)
                {
                    answer=speak_map[keyword+"-medium"];
                }
                else if(t<=30.0)
                {
                    answer=speak_map[keyword+"-low"];
                }
            }

            Bottle &output=speechPortOut.prepare();
            output.clear();
            output.addString(answer);
            speechPortOut.writeStrict();
        }
    }

};

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
class Manager : public RFModule, public managerTUG_IDL
{
    //params
    string module_name;
    string speak_file;
    double period;
    double finish_line_thresh,standing_thresh;

    const int ok=Vocab::encode("ok");
    const int fail=Vocab::encode("fail");
    enum class State { stopped, idle, seek_skeleton, follow, engaged, assess_standing, assess_crossing, line_crossed, not_passed, finished } state;
    string tag;
    double t0,tstart,t;
    int encourage_cnt;
    unordered_map<string,string> speak_map;
    bool interrupting;
    mutex mtx;
    bool start_ex;

    Vector startline_pose,finishline_pose;
    Matrix worldframe;
    Vector coronal;
    bool world_configured;

    //ports
    RpcClient analyzerPort;
    RpcClient speechRpcPort;
    RpcClient attentionPort;
    RpcClient navigationPort;
    BufferedPort<Bottle> speechStreamPort;
    RpcServer cmdPort;
    RpcClient linePort;

    SpeechInterpApi *speech_interp;

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
        if (!bGroup.check("num-sections"))
        {
            yError()<<"Unable to find key \"num-sections\"";
            return false;
        }
        int num_sections=bGroup.find("num-sections").asInt();
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
    bool disengage()
    {
        bool ret=false;
        state=State::idle;

        bool ok_nav=false;
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
                        cmd.clear();
                        rep.clear();
                        cmd.addString("is_navigating");
                        if (navigationPort.write(cmd,rep))
                        {
                            if (rep.get(0).asBool()==true)
                            {
                                cmd.clear();
                                rep.clear();
                                cmd.addString("stop");
                                if (navigationPort.write(cmd,rep))
                                {
                                    if (rep.get(0).asVocab()==ok)
                                    {
                                        ok_nav=true;
                                    }
                                }
                            }
                            else
                            {
                                ok_nav=true;
                            }
                        }
                    }
                }
            }
        }

        if (ok_nav)
        {
            if(finishline_pose.size()>0 && coronal.size()>0)
            {
                cmd.clear();
                rep.clear();
                double x=finishline_pose[0]-0.6;
                double y=finishline_pose[1]-0.6;
                double theta=(180/M_PI)*atan2(-coronal[1],-coronal[0]);
                cmd.addString("go_to_wait");
                cmd.addDouble(x);
                cmd.addDouble(y);
                cmd.addDouble(theta);
                if (navigationPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==ok)
                    {
                        yInfo()<<"Back to initial position";
                        ret=true;
                    }
                }
            }
        }

        t0=Time::now();
        return ret;
    }

    /****************************************************************/
    bool start() override
    {
        lock_guard<mutex> lg(mtx);
        start_ex=true;
        return disengage();
    }

    /****************************************************************/
    bool stop() override
    {
        lock_guard<mutex> lg(mtx);
        bool ret=false;

        bool ok_nav=false;
        Bottle cmd,rep;
        cmd.addString("stop");
        if (attentionPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==ok)
            {
                cmd.clear();
                rep.clear();
                cmd.addString("is_navigating");
                if (navigationPort.write(cmd,rep))
                {
                    if (rep.get(0).asBool()==true)
                    {
                        cmd.clear();
                        rep.clear();
                        cmd.addString("stop");
                        if (navigationPort.write(cmd,rep))
                        {
                            if (rep.get(0).asVocab()==ok)
                            {
                                ok_nav=true;
                            }
                        }
                    }
                    else
                    {
                        ok_nav=true;
                    }
                }
            }
        }

        if (ok_nav)
        {
            if(finishline_pose.size()>0 && coronal.size()>0)
            {
                cmd.clear();
                rep.clear();
                double x=finishline_pose[0]-0.6;
                double y=finishline_pose[1]-0.6;
                double theta=(180/M_PI)*atan2(-coronal[1],-coronal[0]);
                cmd.addString("go_to_wait");
                cmd.addDouble(x);
                cmd.addDouble(y);
                cmd.addDouble(theta);
                if (navigationPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==ok)
                    {
                        yInfo()<<"Back to initial position";
                        ret=true;
                    }
                }
            }
        }

        start_ex=false;
        state=State::stopped;
        return ret;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        module_name=rf.check("name",Value("managerTUG")).asString();
        period=rf.check("period",Value(0.1)).asDouble();
        speak_file=rf.check("speak-file",Value("speak-it")).asString();
        finish_line_thresh=rf.check("finish-line-thresh",Value(0.2)).asDouble();
        standing_thresh=rf.check("standing-thresh",Value(0.3)).asDouble();

        this->setName(module_name.c_str());

        if (!load_speak(rf.getContext(),speak_file))
        {
            string msg="Unable to locate file";
            msg+="\""+speak_file+"\"";
            yError()<<msg;
            return false;
        }
        start_ex=false;

        analyzerPort.open("/"+module_name+"/analyzer:rpc");
        speechRpcPort.open("/"+module_name+"/speech:rpc");
        attentionPort.open("/"+module_name+"/attention:rpc");
        navigationPort.open("/"+module_name+"/navigation:rpc");
        linePort.open("/"+module_name+"/line:rpc");
        speechStreamPort.open("/"+module_name+"/speech:o");
        cmdPort.open("/"+module_name+"/cmd:rpc");
        attach(cmdPort);

        speech_interp=new SpeechInterpApi(module_name,speak_map);
        speech_interp->open();

        state=State::idle;
        interrupting=false;
        world_configured=false;
        t0=tstart=Time::now();

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
        lock_guard<mutex> lg(mtx);
        if((analyzerPort.getOutputCount()==0) || (speechStreamPort.getOutputCount()==0) ||
                (speechRpcPort.getOutputCount()==0) || (attentionPort.getOutputCount()==0) ||
                (navigationPort.getOutputCount()==0) || (linePort.getOutputCount())==0)
        {
            yInfo()<<"not connected";
            return true;
        }

        if(!start_ex)
        {
            return true;
        }

        //get start and finish lines
        if (!world_configured)
        {
            yInfo()<<"Getting world...";
            world_configured=getWorld();
            return true;
        }

        string follow_tag("");
        {
            Bottle cmd,rep;
            cmd.addString("is_following");
            if (attentionPort.write(cmd,rep))
            {
                follow_tag=rep.get(0).asString();
            }
        }

        if (state==State::idle)
        {
            if (Time::now()-t0>10.0)
            {
                state=State::seek_skeleton;
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

        if (state==State::seek_skeleton)
        {
            if (!follow_tag.empty())
            {
                tag=follow_tag;
                Bottle cmd,rep;
                cmd.addString("look");
                cmd.addString(tag);
                cmd.addString(KeyPointTag::shoulder_center);
                if (attentionPort.write(cmd,rep))
                {
                    yInfo()<<"Following"<<follow_tag;
                    vector<SpeechParam> p;
                    p.push_back(SpeechParam(tag[0]!='#'?tag:string("")));
                    speak("invite-start",true,p);
                    speak("engage",true);
                    state=State::follow;
                    encourage_cnt=0;
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
            }
        }

        if (state==State::engaged)
        {
            Bottle cmd,rep;
            cmd.addString("loadExercise");
            cmd.addString("tug");
            if (analyzerPort.write(cmd,rep))
            {
                bool ack=rep.get(0).asBool();
                if (ack)
                {
                    cmd.clear();
                    rep.clear();
                    cmd.addString("listMetrics");
                    if (analyzerPort.write(cmd,rep))
                    {
                        Bottle &metrics=*rep.get(0).asList();
                        if (metrics.size()>0)
                        {
                            string metric="step_0";//select_randomly(metrics);
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
                                            string prop="step_length";//select_randomly(props);
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
                                                    cmd.addString("selectSkel");
                                                    cmd.addString(tag);
                                                    yInfo()<<"Selecting skeleton"<<tag;
                                                    if (analyzerPort.write(cmd,rep))
                                                    {
                                                        if (rep.get(0).asVocab()==ok)
                                                        {
                                                            speak("sit",true);
                                                            speak("explain",true);
                                                            cmd.clear();
                                                            rep.clear();
                                                            cmd.addString("is_following");
                                                            if (attentionPort.write(cmd,rep))
                                                            {
                                                                if (!rep.get(0).asString().empty())
                                                                {
                                                                    if (Bottle *corB=rep.get(4).asList())
                                                                    {
                                                                        coronal.clear();
                                                                        coronal.resize(3);
                                                                        coronal[0]=corB->get(0).asDouble();
                                                                        coronal[1]=corB->get(1).asDouble();
                                                                        coronal[2]=corB->get(2).asDouble();

                                                                        double x=finishline_pose[0]-0.3;
                                                                        double y=finishline_pose[1]-0.2;
                                                                        double theta=(180/M_PI)*atan2(-coronal[1],-coronal[0]);
                                                                        cmd.clear();
                                                                        rep.clear();
                                                                        cmd.addString("go_to_wait");
                                                                        cmd.addDouble(x);
                                                                        cmd.addDouble(y);
                                                                        cmd.addDouble(theta);
                                                                        if (navigationPort.write(cmd,rep))
                                                                        {
                                                                            if (rep.get(0).asVocab()==ok)
                                                                            {
                                                                                speak("line",true);
                                                                                speak("questions",true);
                                                                                double t1=Time::now();
                                                                                while(speech_interp->getAnswer().empty())
                                                                                {
                                                                                    double timeout=Time::now()-t1;
                                                                                    if(timeout>10.0)
                                                                                        break;
                                                                                }

                                                                                cmd.clear();
                                                                                rep.clear();
                                                                                cmd.addString("track_skeleton");
                                                                                cmd.addString(tag);
                                                                                if (navigationPort.write(cmd,rep))
                                                                                {
                                                                                    if (rep.get(0).asVocab()==ok)
                                                                                    {
                                                                                        cmd.clear();
                                                                                        rep.clear();
                                                                                        cmd.addString("look");
                                                                                        cmd.addString(tag);
                                                                                        cmd.addString(KeyPointTag::hip_center);
                                                                                        if (attentionPort.write(cmd,rep))
                                                                                        {
                                                                                            speak("start",true);
                                                                                            cmd.clear();
                                                                                            rep.clear();
                                                                                            cmd.addString("start");
                                                                                            cmd.addInt(1);
                                                                                            if (analyzerPort.write(cmd,rep))
                                                                                            {
                                                                                                if (rep.get(0).asVocab()==ok)
                                                                                                {
                                                                                                    state=State::assess_standing;
                                                                                                    t0=tstart=Time::now();
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
                    }
                }
            }
        }

        speech_interp->setTime(Time::now()-tstart);
        if (state==State::assess_standing)
        {
            //check if the person stands up
            Bottle cmd,rep;
            cmd.addString("isStanding");
            cmd.addDouble(standing_thresh);
            if (analyzerPort.write(cmd,rep))
            {
                if(rep.get(0).asVocab()==ok)
                {
                    yInfo()<<"Person standing";
                    state=State::assess_crossing;
                    encourage_cnt=0;
                    t0=Time::now();
                }
                else
                {
                    if((Time::now()-t0)>10.0)
                    {
                        if(++encourage_cnt<=1)
                        {
                            speak("encourage",false);
                            t0=Time::now();
                        }
                        else
                        {
                            state=State::not_passed;
                        }
                    }
                }
            }
        }

        if (state==State::assess_crossing)
        {
            Bottle cmd,rep;
            cmd.addString("hasCrossedFinishLine");
            cmd.addDouble(finish_line_thresh);
            if (analyzerPort.write(cmd,rep))
            {
                if(rep.get(0).asVocab()==ok)
                {
                    yInfo()<<"Line crossed!";
                    state=State::line_crossed;
                    encourage_cnt=0;
                    t0=Time::now();
                }
                else
                {
                    cmd.clear();
                    rep.clear();
                    cmd.addString("isSitting");
                    cmd.addDouble(standing_thresh);
                    if (analyzerPort.write(cmd,rep))
                    {
                        if(rep.get(0).asVocab()==ok)
                        {
                            yInfo()<<"Test finished but line not crossed";
                            speak("not-crossed",true);
                            state=State::not_passed;
                        }
                        else
                        {
                            if((Time::now()-t0)>30.0)
                            {
                                if(++encourage_cnt<=1)
                                {
                                    speak("encourage",false);
                                    t0=Time::now();
                                }
                                else
                                {
                                    state=State::not_passed;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (state==State::line_crossed)
        {
            //detect when the person seats down
            Bottle cmd,rep;
            cmd.addString("isSitting");
            cmd.addDouble(standing_thresh);
            if (analyzerPort.write(cmd,rep))
            {
                if(rep.get(0).asVocab()==ok)
                {
                    t=Time::now()-tstart;
                    yInfo()<<"Test finished in"<<t<<"seconds";
                    state=State::finished;
                }
                else
                {
                    if((Time::now()-t0)>30.0)
                    {
                        if(++encourage_cnt<=1)
                        {
                            speak("encourage",false);
                            t0=Time::now();
                        }
                        else
                        {
                            state=State::not_passed;
                        }
                    }
                }
            }
        }

        if (state==State::finished)
        {
            vector<SpeechParam> p;
            p.push_back(round(t*10.0)/10.0);
            Bottle cmd,rep;
            cmd.addString("stop");
            analyzerPort.write(cmd,rep);
            speak("assess-high",true,p);
            speak("greetings",true);
            disengage();
        }

        if (state==State::not_passed)
        {
            Bottle cmd,rep;
            cmd.addString("stop");
            analyzerPort.write(cmd,rep);
            speak("end",true);
            speak("assess-low",true);
            speak("greetings",true);
            disengage();
        }

        return true;
    }

    /****************************************************************/
    bool getWorld()
    {
        Bottle cmd,rep;
        cmd.addString("get_world_frame");
        if(linePort.write(cmd,rep))
        {
            if(Bottle* worldBottle=rep.get(0).asList())
            {
                if(worldBottle->size()>=3 && worldBottle->get(0).asInt()>=4 && worldBottle->get(1).asInt()>=4)
                {
                    if(Bottle *wfB=worldBottle->get(2).asList())
                    {
                        if(wfB->size()>=16)
                        {
                            Matrix worldframe(4,4);
                            worldframe(0,0)=wfB->get(0).asDouble();
                            worldframe(0,1)=wfB->get(1).asDouble();
                            worldframe(0,2)=wfB->get(2).asDouble();
                            worldframe(0,3)=wfB->get(3).asDouble();

                            worldframe(1,0)=wfB->get(4).asDouble();
                            worldframe(1,1)=wfB->get(5).asDouble();
                            worldframe(1,2)=wfB->get(6).asDouble();
                            worldframe(1,3)=wfB->get(7).asDouble();

                            worldframe(2,0)=wfB->get(8).asDouble();
                            worldframe(2,1)=wfB->get(9).asDouble();
                            worldframe(2,2)=wfB->get(10).asDouble();
                            worldframe(2,3)=wfB->get(11).asDouble();

                            worldframe(3,0)=wfB->get(12).asDouble();
                            worldframe(3,1)=wfB->get(13).asDouble();
                            worldframe(3,2)=wfB->get(14).asDouble();
                            worldframe(3,3)=wfB->get(15).asDouble();

                            cmd.clear();
                            rep.clear();
                            cmd.addString("get_state");
                            if (navigationPort.write(cmd,rep))
                            {
                                Property robotState(rep.get(0).toString().c_str());
                                if (Bottle *loc=robotState.find("robot-location").asList())
                                {
                                    Vector robot_location(4);
                                    robot_location[0]=loc->get(0).asDouble();
                                    robot_location[1]=loc->get(1).asDouble();
                                    robot_location[2]=0.0;
                                    robot_location[3]=1.0;
                                    robot_location=worldframe*robot_location;
                                    robot_location.pop_back();

                                    Vector rot=dcm2axis(worldframe.submatrix(0,2,0,2));
                                    cmd.clear();
                                    rep.clear();
                                    cmd.addString("reset_odometry");
                                    cmd.addDouble(robot_location[0]);
                                    cmd.addDouble(robot_location[1]);
                                    cmd.addDouble(-(180/M_PI)*rot[3]);
                                    if (navigationPort.write(cmd,rep))
                                    {
                                        if (rep.get(0).asVocab()==ok)
                                        {
                                            yInfo()<<cmd.toString();
                                            cmd.clear();
                                            rep.clear();
                                            cmd.addString("get_line_pose");
                                            cmd.addString("finish-line");
                                            if(linePort.write(cmd,rep))
                                            {
                                                if(Bottle *lp_bottle=rep.get(0).asList())
                                                {
                                                    if(lp_bottle->size()>=7)
                                                    {
                                                        finishline_pose.resize(7);
                                                        finishline_pose[0]=lp_bottle->get(0).asDouble();
                                                        finishline_pose[1]=lp_bottle->get(1).asDouble();
                                                        finishline_pose[2]=lp_bottle->get(2).asDouble();
                                                        finishline_pose[3]=lp_bottle->get(3).asDouble();
                                                        finishline_pose[4]=lp_bottle->get(4).asDouble();
                                                        finishline_pose[5]=lp_bottle->get(5).asDouble();
                                                        finishline_pose[6]=lp_bottle->get(6).asDouble();
                                                        yInfo()<<"Finish line wrt world frame"<<finishline_pose.toString();

                                                        cmd.clear();
                                                        rep.clear();
                                                        cmd.addString("setLinePose");
                                                        cmd.addList().read(finishline_pose);
                                                        if(analyzerPort.write(cmd,rep))
                                                        {
                                                            yInfo()<<"Set finish line to motionAnalyzer";
                                                            return true;
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
        yError()<<"Could not configure world";

        return false;
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
        speech_interp->interrupt();
        speech_interp->close();
        delete speech_interp;
        analyzerPort.close();
        speechRpcPort.close();
        attentionPort.close();
        navigationPort.close();
        linePort.close();
        speechStreamPort.close();
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
    rf.setDefaultContext("managerTUG");
    rf.setDefaultConfigFile("config-it.ini");
    rf.configure(argc,argv);

    Manager manager;
    return manager.runModule(rf);
}


