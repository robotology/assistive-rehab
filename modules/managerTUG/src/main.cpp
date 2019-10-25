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
    unordered_map<string,string> speak_map;
    double t;
    string keyword;

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
        return true;
    }

    /********************************************************/
    void close()
    {
        BufferedPort<yarp::os::Bottle >::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle >::interrupt();
    }

    /********************************************************/
    void setTime(const double &t)
    {
        this->t=t;
    }

    /********************************************************/
    string getKeyword() const
    {
        return keyword;
    }

    /********************************************************/
    void resetKeyword()
    {
        keyword.clear();
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &speech_interpretation )
    {
        yInfo()<<"Received a question...";
        if(!speech_interpretation.isNull())
        {
            yInfo()<<"Trying to answer...";
            keyword=speech_interpretation.get(0).asString();
            if (!keyword.empty())
            {
                if(keyword=="feedback")
                {
                    if(t<=10.0)
                    {
                        keyword+="-high";
                    }
                    else if(t<=20.0)
                    {
                        keyword+="-medium";
                    }
                    else if(t<=30.0)
                    {
                        keyword+="-low";
                    }
                }
                yInfo()<<"Received keyword"<<keyword;
            }
            if (keyword.empty() || speak_map.count(keyword)<=0)
            {
                yInfo()<<"Could not understand the question";
                yInfo()<<"Asking to repeat the question";
                keyword="unclear";
            }
        }
        else
        {
            yWarning()<<"Corrupted bottle";
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

    Vector startline_pose,finishline_pose,starting_pose;
    Matrix worldframe;
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
                            if (rep.get(0).asVocab()==ok)
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
            cmd.clear();
            rep.clear();
            cmd.addString("go_to_wait");
            cmd.addDouble(starting_pose[0]);
            cmd.addDouble(starting_pose[1]);
            cmd.addDouble(starting_pose[2]);
            if (navigationPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==ok)
                {
                    yInfo()<<"Back to initial position";
                    ret=true;
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
        bool ret=false;
        Bottle cmd,rep;
        cmd.addString("stop");
        if (attentionPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==ok)
            {
                cmd.clear();
                rep.clear();
                cmd.addString("set_auto");
                if (attentionPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==ok)
                    {
                        cmd.clear();
                        rep.clear();
                        cmd.addString("go_to_wait");
                        cmd.addDouble(starting_pose[0]);
                        cmd.addDouble(starting_pose[1]);
                        cmd.addDouble(starting_pose[2]);
                        if (navigationPort.write(cmd,rep))
                        {
                            if (rep.get(0).asVocab()==ok)
                            {
                                yInfo()<<"Going to starting position";
                                ret=true;
                            }
                        }
                    }
                }
            }
        }

        start_ex=ret;
        return start_ex;
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
            cmd.clear();
            rep.clear();
            cmd.addString("go_to_wait");
            cmd.addDouble(starting_pose[0]);
            cmd.addDouble(starting_pose[1]);
            cmd.addDouble(starting_pose[2]);
            if (navigationPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==ok)
                {
                    yInfo()<<"Back to initial position";
                    ret=true;
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
        finish_line_thresh=rf.check("finish-line-thresh",Value(0.3)).asDouble();
        standing_thresh=rf.check("standing-thresh",Value(0.2)).asDouble();
        starting_pose={1.5,-3.0,110.0};
        if(rf.check("starting-pose"))
        {
            if (const Bottle *sp=rf.find("starting-pose").asList())
            {
                size_t len=sp->size();
                for (size_t i=0; i<len; i++)
                {
                    starting_pose[i]=sp->get(i).asDouble();
                }
            }
        }

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

        //get start and finish lines
        if (!world_configured)
        {
            yInfo()<<"Getting world...";
            world_configured=getWorld();
            return true;
        }

        if(!start_ex)
        {
            return true;
        }

        {
            string kw=speech_interp->getKeyword();
            reply(kw);
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
                                                                    double x=finishline_pose[0]+0.2;
                                                                    double y=finishline_pose[1]-0.5;
                                                                    double theta=starting_pose[2];
                                                                    cmd.clear();
                                                                    rep.clear();
                                                                    cmd.addString("go_to_wait");
                                                                    cmd.addDouble(x);
                                                                    cmd.addDouble(y);
                                                                    cmd.addDouble(theta);
                                                                    yDebug()<<cmd.toString();
                                                                    if (navigationPort.write(cmd,rep))
                                                                    {
                                                                        if (rep.get(0).asVocab()==ok)
                                                                        {
                                                                            speak("line",true);
                                                                            speak("questions",true);
                                                                            double t1=Time::now();
                                                                            bool gotQuestion=false;
                                                                            while(!gotQuestion)
                                                                            {
                                                                                string kw=speech_interp->getKeyword();
                                                                                gotQuestion=reply(kw);
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
    bool reply(const string &kw)
    {
        bool ret=false;
        if (!kw.empty())
        {
            yInfo()<<"Interrupting modules";
            freeze();
            ret=speak(kw,true);
            speech_interp->resetKeyword();
            yInfo()<<"Restoring modules";
            restore();
        }
        return ret;
    }

    /****************************************************************/
    bool freeze()
    {
        Bottle cmd,rep;
        cmd.addString("stop");
        if(navigationPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==ok)
            {
                yInfo()<<"navController stopped";
                cmd.clear();
                rep.clear();
                cmd.addString("freeze");
                if(analyzerPort.write(cmd,rep))
                {
                    if(rep.get(0).asVocab()==ok)
                    {
                        yInfo()<<"motionAnalyzer frozen";
                        return true;
                    }
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool restore()
    {
        Bottle cmd,rep;
        cmd.addString("start");
        if(analyzerPort.write(cmd,rep))
        {
            if(rep.get(0).asBool()==true)
            {
                yInfo()<<"motionAnalyzer restored";
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool getWorld()
    {
        Bottle cmd,rep;
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


