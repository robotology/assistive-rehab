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

#include <condition_variable>

#include <mutex>
#include <cmath>

#include <AssistiveRehab/skeleton.h>

#include "src/managerTUG_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class TriggerManager: public Thread
{
    string module_name;
    double min_timeout;
    double max_timeout;
    double t0,t;
    RpcClient *triggerPort;
    bool first_trigger,last_start_trigger,got_trigger;
    mutex mtx;

public:

    /********************************************************/
    TriggerManager(const string &module_name, const double &min_timeout, const double &max_timeout, RpcClient *triggerPort)
    {
        this->module_name=module_name;
        this->min_timeout=min_timeout;
        this->max_timeout=max_timeout;
        this->triggerPort=triggerPort;
        t0=t=Time::now();
        first_trigger=true;
        last_start_trigger=false;
        got_trigger=false;
    }

    /********************************************************/
    ~TriggerManager()
    {
    }

    /********************************************************/
    void run() override
    {
        while(!isStopping())
        {
            lock_guard<mutex> lg(mtx);
            double dt_from_last=t-t0;
            double time_elapsed=Time::now()-t;

            //if we received a trigger
            if (got_trigger)
            {
                got_trigger=false;
                if (first_trigger)
                {
                    //if it's the first trigger, we assume it's a start
                    first_trigger=false;
                    trigger_speech("start");
                    t0=t;
                    continue;
                }

                //if trigger occurs after the minimum timeout
                if (dt_from_last>=min_timeout)
                {
                    //if last was a start trigger
                    if (last_start_trigger)
                    {
                        //we send a stop
                        last_start_trigger=false;
                        trigger_speech("stop");
                    }
                    else //if last was a stop trigger
                    {
                        //we send a start
                        last_start_trigger=true;
                        trigger_speech("start");
                    }
                    t0=t;
                }
                else
                {
                    yInfo()<<"Trigger occurred before min timeout";
                    yInfo()<<"Discarding trigger";
                }
                continue;
            }

            //if we haven't received a trigger in the last max_timeout seconds
            //and last trigger received was a start
            if (last_start_trigger)
            {
                //we send a stop
                if (time_elapsed>max_timeout)
                {
                    last_start_trigger=false;
                    yInfo()<<"Exceeded max timeout";
                    trigger_speech("stop");
                    t0=t;
                }
            }
        }
    }

    /********************************************************/
    bool trigger_speech(const string &s)
    {
        yarp::os::Bottle cmd,rep;
        cmd.addString(s);
        if (triggerPort->write(cmd,rep))
        {
            if (rep.get(0).asVocab()==Vocab::encode("ok"))
            {
                yInfo()<<"Sending"<<s<<"to speech";
                return true;
            }
        }
        return false;
    }

    /********************************************************/
    void trigger()
    {
        lock_guard<mutex> lg(mtx);
        t=Time::now();
        got_trigger=true;
        if (first_trigger)
        {
            t0=t;
            last_start_trigger=true;
        }
        yInfo()<<"Got button trigger from the previous after"<<t-t0<<"seconds";
    }

};

/********************************************************/
class QuestionManager: public BufferedPort<Bottle>
{
    string module_name;
    unordered_map<string,string> speak_map;
    BufferedPort<Bottle> answerPort;
    BufferedPort<Bottle> *speechPort;
    RpcClient *speechRpc;
    bool freezing,asked,replied,silent;
    double time;
    mutex mtx;

public:

    /********************************************************/
    QuestionManager(const string &module_name, const unordered_map<string,string> &speak_map,
                    BufferedPort<Bottle> *speechPort, RpcClient *speechRpc)
    {
        this->module_name=module_name;
        this->speak_map=speak_map;
        this->speechPort=speechPort;
        this->speechRpc=speechRpc;
        freezing=false;
        asked=false;
        replied=false;
        silent=true;
    }

    /********************************************************/
    ~QuestionManager()
    {
    };

    /********************************************************/
    bool open()
    {
        this->useCallback();
        BufferedPort<yarp::os::Bottle >::open("/"+module_name+"/question:i");
        answerPort.open("/"+module_name+"/answer:i");
        return true;
    }

    /********************************************************/
    bool connected()
    {
        if(answerPort.getInputCount() < 1 || this->getInputCount() < 1)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    /********************************************************/
    void close()
    {
        BufferedPort<yarp::os::Bottle >::close();
        answerPort.close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle >::interrupt();
        answerPort.interrupt();
    }

    /********************************************************/
    bool freeze() const
    {
        return freezing;
    }

    /********************************************************/
    bool restore() const
    {
        return !freezing;
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &question )
    {
        lock_guard<mutex> lg(mtx);
        if(!silent)
        {
            yInfo()<<"Received a question...";
            freezing=true;
            asked=true;
            replied=false;
            reply(speak_map["asking"]);
            string keyword;
            bool got_kw=false;
            while (true)
            {
                got_kw=getAnswer(keyword);
                if (got_kw)
                {
                    break;
                }
            }
            string answer=speak_map[keyword];
            replied=reply(answer);
            freezing=false;
            asked=false;
        }
    }

    /********************************************************/
    void wakeUp()
    {
        silent=false;
    }

    /********************************************************/
    void suspend()
    {
        silent=true;
    }

    /********************************************************/
    void setTime(const double &t)
    {
        this->time=t;
    }

    /********************************************************/
    bool getAnswer(string &keyword)
    {
        yarp::os::Bottle *speech_interpretation=answerPort.read(true);
        if(!speech_interpretation->isNull())
        {
            yInfo()<<"Trying to answer...";
            keyword=speech_interpretation->get(0).asString();
            if (!keyword.empty())
            {
                if(keyword=="feedback")
                {
                    if(time<=10.0)
                    {
                        keyword+="-high";
                    }
                    else if(time<=30.0)
                    {
                        keyword+="-medium";
                    }
                    else if(time>=30.0)
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
            return false;
        }

        return true;
    }

    /********************************************************/
    bool reply(const string &ans)
    {
        bool ret=false;
        Bottle &answer=speechPort->prepare();
        answer.clear();
        answer.addString(ans);
        speechPort->writeStrict();
        while (true && (speechPort->getOutputCount()>0))
        {
            Time::delay(0.01);
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("stat"));
            if (speechRpc->write(cmd,rep))
            {
                if (rep.get(0).asString()=="quiet")
                {
                    ret=true;
                    break;
                }
            }
        }
        Time::delay(0.1);

        return ret;
    }

    /********************************************************/
    bool gotQuestion()
    {
        lock_guard<mutex> lg(mtx);
        return asked;
    }

    /********************************************************/
    bool hasReplied()
    {
        lock_guard<mutex> lg(mtx);
        return replied;
    }
};

/****************************************************************/
class HandManager : public Thread
{
    string module_name;
    BufferedPort<Bottle> *opcPort;
    RpcClient *triggerPort;
    Skeleton *skeleton;
    double arm_thresh;
    string tag,part;
    bool isArmLifted;

public:

    /********************************************************/
    HandManager(const string &module_name, const double &arm_thresh, BufferedPort<Bottle> *opcPort, RpcClient *triggerPort)
    {
        this->module_name=module_name;
        this->arm_thresh=arm_thresh;
        this->opcPort=opcPort;
        this->triggerPort=triggerPort;
        isArmLifted=false;
        tag="";
        part="";
    }

    /********************************************************/
    void set_tag(const string &tag)
    {
        this->tag=tag;
    }

    /********************************************************/
    void run() override
    {
        while(!isStopping())
        {
            if (opcPort->getInputCount() > 0)
            {
                if (!tag.empty())
                {
                    get_skeleton();
                    if (isArmLifted==false)
                    {
                        if (is_with_raised_hand())
                        {
                            yarp::os::Bottle cmd,rep;
                            cmd.addString("start");
                            isArmLifted=true;
                            triggerPort->write(cmd,rep);
                            if (rep.get(0).asVocab()==Vocab::encode("ok"))
                            {
                                yInfo()<<"Starting speech";
                            }
                        }
                    }
                    else
                    {
                        if (!is_raised())
                        {
                            yarp::os::Bottle cmd,rep;
                            cmd.addString("stop");
                            isArmLifted=false;
                            triggerPort->write(cmd,rep);
                            if (rep.get(0).asVocab()==Vocab::encode("ok"))
                            {
                                yInfo()<<"Stopping speech";
                            }
                        }
                    }
                }
            }
        }
    }

    /****************************************************************/
    void onStop() override
    {
        delete skeleton;
    }

    /****************************************************************/
    void get_skeleton()
    {
        if (Bottle* b=opcPort->read(true))
        {
            if (!b->get(1).isString())
            {
                for (int i=1; i<b->size(); i++)
                {
                    Property prop;
                    prop.fromString(b->get(i).asList()->toString());
                    string skel_tag=prop.find("tag").asString();
                    if (skel_tag==tag)
                    {
                        skeleton=skeleton_factory(prop);
                        skeleton->normalize();
                    }
                }
            }
        }
    }

    /****************************************************************/
    bool is_with_raised_hand()
    {
        if (is_raised("left"))
        {
            part="left";
            return true;
        }
        if (is_raised("right"))
        {
            part="right";
            return true;
        }

        return false;
    }

    /****************************************************************/
    bool is_raised() const
    {
        return is_raised(part);
    }

    /****************************************************************/
    bool is_raised(const string &p) const
    {
        string elbow=(p=="left"?KeyPointTag::elbow_left:KeyPointTag::elbow_right);
        string hand=(p=="left"?KeyPointTag::hand_left:KeyPointTag::hand_right);
        if (!p.empty())
        {
            if ((*skeleton)[elbow]->isUpdated() &&
                (*skeleton)[hand]->isUpdated())
            {
                if (((*skeleton)[hand]->getPoint()[2]-(*skeleton)[elbow]->getPoint()[2])>arm_thresh)
                {
                    return true;
                }
            }
        }

        return false;
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
    double finish_line_thresh,standing_thresh,pointing_time,arm_thresh;
    bool detect_hand_up;
    Vector starting_pose,pointing_home,pointing_start,pointing_finish;
    double min_timeout,max_timeout;

    const int ok=Vocab::encode("ok");
    const int fail=Vocab::encode("fail");
    enum class State { stopped, idle, lock, seek_locked, follow, assess_standing, assess_crossing, line_crossed, frozen, engaged, explain, reach_line, starting, not_passed, finished } state;
    State prev_state;
    string tag;
    double t0,tstart,t;
    int encourage_cnt,reinforce_engage_cnt;
    unordered_map<string,string> speak_map;
    unordered_map<string,int> speak_count_map;
    bool interrupting;
    mutex mtx;
    bool start_ex,ok_go,connected;

    Vector finishline_pose;
    double line_length;
    bool world_configured;

    //ports
    RpcClient analyzerPort;
    RpcClient speechRpcPort;
    RpcClient attentionPort;
    RpcClient navigationPort;
    BufferedPort<Bottle> speechStreamPort;
    RpcServer cmdPort;
    RpcClient leftarmPort;
    RpcClient rightarmPort;
    BufferedPort<Bottle> opcPort;
    RpcClient lockerPort;
    RpcClient triggerPort;

    QuestionManager *question_manager;
    HandManager *hand_manager;
    TriggerManager *trigger_manager;

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
            speak_count_map[key]=0;
        }

        return true;
    }

    /****************************************************************/
    bool speak(const string &key, const bool wait,
               const vector<SpeechParam> &p=vector<SpeechParam>())
    {
        //if a question was received, we wait until an answer is given, before speaking
        bool received_question=question_manager->gotQuestion();
        if (received_question)
        {
            bool can_speak=false;
            yInfo()<<"Replying to question first";
            while (true)
            {
                can_speak=question_manager->hasReplied();
                if (can_speak)
                {
                    break;
                }
            }
        }
        if (speak_count_map[key]>0)
        {
            yInfo()<<"Skipping"<<key;
            return true;
        }
        yInfo()<<"Speaking"<<key;

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

        speak_count_map[key]+=1;
        return (it!=end(speak_map));
    }

    /****************************************************************/
    bool disengage()
    {
        bool ret=false;
        state=State::idle;
        ok_go=false;
        question_manager->suspend();

        for (auto it=speak_map.begin(); it!=speak_map.end(); it++)
        {
            string key=it->first;
            speak_count_map[key]=0;
        }

        bool ok_nav=false;
        Bottle cmd,rep;
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

        if (ok_nav)
        {
            if (go_to(starting_pose,true))
            {
                yInfo()<<"Back to initial position";
                cmd.clear();
                rep.clear();
                cmd.addString("set_auto");
                if (attentionPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==ok)
                    {
                        ret=remove_locked();
                    }
                }
            }
        }

        t0=Time::now();
        return ret;
    }

    /****************************************************************/
    bool go_to(const Vector &target, const bool &wait)
    {
        Bottle cmd,rep;
        if (wait)
            cmd.addString("go_to_wait");
        else
            cmd.addString("go_to");
        cmd.addDouble(target[0]);
        cmd.addDouble(target[1]);
        cmd.addDouble(target[2]);
        yDebug()<<cmd.toString();
        if (navigationPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==ok)
            {
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool remove_locked()
    {
        Bottle cmd,rep;
        cmd.addString("remove_locked");
        if (lockerPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==ok)
            {
                yInfo()<<"Removed locked skeleton";
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool start() override
    {
        lock_guard<mutex> lg(mtx);
        if (!connected)
        {
            yError()<<"Not connected";
            return false;
        }
        state=State::idle;
        bool ret=false;
        if (go_to(starting_pose,true))
        {
            yInfo()<<"Going to initial position";
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
                            ret=true;
                        }
                    }
                }
            }
        }

        start_ex=ret;
        return start_ex;
    }

    /****************************************************************/
    bool trigger() override
    {
        lock_guard<mutex> lg(mtx);
        trigger_manager->trigger();
        return true;
    }

    /****************************************************************/
    bool stop() override
    {
        lock_guard<mutex> lg(mtx);
        bool ret=disengage();
        Bottle cmd,rep;
        cmd.addString("stop");
        if (attentionPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==fail)
            {
                return false;
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
        standing_thresh=rf.check("standing-thresh",Value(0.2)).asDouble();
        arm_thresh=rf.check("arm-thresh",Value(0.6)).asDouble();
        detect_hand_up=rf.check("detect-hand-up",Value(false)).asBool();
        min_timeout=rf.check("min-timeout",Value(1.0)).asDouble();
        max_timeout=rf.check("max-timeout",Value(10.0)).asDouble();
        starting_pose={1.5,-3.0,110.0};
        if(rf.check("starting-pose"))
        {
            if (Bottle *sp=rf.find("starting-pose").asList())
            {
                size_t len=sp->size();
                for (size_t i=0; i<len; i++)
                {
                    starting_pose[i]=sp->get(i).asDouble();
                }
            }
        }

        pointing_time=rf.check("pointing-time",Value(3.0)).asDouble();
        pointing_home={-10.0,20.0,-10.0,35.0,0.0,0.030,0.0,0.0};
        if(rf.check("pointing-home"))
        {
            if (Bottle *ph=rf.find("pointing-home").asList())
            {
                size_t len=ph->size();
                for (size_t i=0; i<len; i++)
                {
                    pointing_home[i]=ph->get(i).asDouble();
                }
            }
        }

        pointing_start={70.0,12.0,-10.0,10.0,0.0,0.050,0.0,0.0};
        if(rf.check("pointing-start"))
        {
            if (Bottle *ps=rf.find("pointing-start").asList())
            {
                size_t len=ps->size();
                for (size_t i=0; i<len; i++)
                {
                    pointing_start[i]=ps->get(i).asDouble();
                }
            }
        }

        pointing_finish={35.0,12.0,-10.0,10.0,0.0,0.050,0.0,0.0};
        if(rf.check("pointing-finish"))
        {
            if (Bottle *pf=rf.find("pointing-finish").asList())
            {
                size_t len=pf->size();
                for (size_t i=0; i<len; i++)
                {
                    pointing_finish[i]=pf->get(i).asDouble();
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
        speechStreamPort.open("/"+module_name+"/speech:o");
        leftarmPort.open("/"+module_name+"/left_arm:rpc");
        rightarmPort.open("/"+module_name+"/right_arm:rpc");
        cmdPort.open("/"+module_name+"/cmd:rpc");
        opcPort.open("/"+module_name+"/opc:i");
        lockerPort.open("/"+module_name+"/locker:rpc");
        triggerPort.open("/"+module_name+"/trigger:rpc");
        attach(cmdPort);

        question_manager=new QuestionManager(module_name,speak_map,&speechStreamPort,&speechRpcPort);
        if (!question_manager->open())
        {
            yError()<<"Could not open question manager";
            return false;
        }

        if(detect_hand_up)
        {
            hand_manager=new HandManager(module_name,arm_thresh,&opcPort,&triggerPort);
            if (!hand_manager->start())
            {
                yError()<<"Could not start hand manager";
                return false;
            }
        }

        trigger_manager=new TriggerManager(module_name,min_timeout,max_timeout,&triggerPort);
        if (!trigger_manager->start())
        {
            yError()<<"Could not open trigger manager";
            return false;
        }

        state=State::idle;
        interrupting=false;
        world_configured=false;
        ok_go=false;
        connected=false;
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
                (navigationPort.getOutputCount()==0) || (leftarmPort.getOutputCount()==0) ||
                (rightarmPort.getOutputCount())==0 || (lockerPort.getOutputCount()==0) ||
                (opcPort.getInputCount()==0) || (triggerPort.getOutputCount()==0) ||
                !question_manager->connected())
        {
            yInfo()<<"not connected";
            connected=true;
            return true;
        }

        //get finish line
        Property l;
        if (hasLine(l))
        {
            if (!world_configured)
            {
                getWorld(l);
            }
        }
        else
        {
            world_configured=false;
            yInfo()<<"World not configured";
            return true;
        }

        if(!start_ex)
        {
            return true;
        }

        if (state==State::frozen)
        {
            if (question_manager->restore())
            {
                state=prev_state;
            }
        }

        if (state>State::frozen)
        {
            if (question_manager->freeze())
            {
                prev_state=state;
                if (prev_state==State::reach_line)
                {
                    Bottle cmd,rep;
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
                                    yInfo()<<"Frozen navigation";
                                }
                            }
                        }
                    }
                }
                state=State::frozen;
            }
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
                state=State::lock;
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

        if (state==State::lock)
        {
            if (!follow_tag.empty())
            {
                Bottle cmd,rep;
                cmd.addString("set_skeleton_tag");
                cmd.addString(follow_tag);
                if (lockerPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==ok)
                    {
                        yInfo()<<"skeleton"<<follow_tag<<"locked";
                        state=State::seek_locked;
                    }
                }
            }
        }

        if (state==State::seek_locked)
        {
            tag=findLocked();
            if (!tag.empty())
            {
                Bottle cmd,rep;
                cmd.addString("look");
                cmd.addString(tag);
                cmd.addString(KeyPointTag::shoulder_center);
                if (attentionPort.write(cmd,rep))
                {
                    yInfo()<<"Following"<<tag;
                    vector<SpeechParam> p;
                    p.push_back(SpeechParam(tag[0]!='#'?tag:string("")));
                    speak("invite-start",true,p);
                    speak("engage",true);
                    state=State::follow;
                    encourage_cnt=0;
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
                    speak("questions",true);
                    if (detect_hand_up)
                    {
                        hand_manager->set_tag(tag);
                    }
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
            question_manager->wakeUp();
            Bottle cmd,rep;
            cmd.addString("setLinePose");
            cmd.addList().read(finishline_pose);
            if(analyzerPort.write(cmd,rep))
            {
                yInfo()<<"Set finish line to motionAnalyzer";
                cmd.clear();
                rep.clear();
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
                                                                state=State::explain;
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

        if (state==State::explain)
        {
            string part=which_part();
            point(pointing_start,part,true);
            speak("sit",true);
            point(pointing_home,part,false);
            speak("explain-start",true);
            speak("explain-walk",true);
            Bottle cmd,rep;
            cmd.addString("is_following");
            if (attentionPort.write(cmd,rep))
            {
                if (!rep.get(0).asString().empty())
                {
                    state=State::reach_line;
                }
            }
        }

        if (state==State::reach_line)
        {
            bool navigating;
            Bottle cmd,rep;
            cmd.addString("is_navigating");
            if (navigationPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()!=ok)
                {
                    navigating=false;
                }
                else
                {
                    navigating=true;
                }
            }

            if (!navigating)
            {
                string part=which_part();
                double x=finishline_pose[0]+0.2;
                if (part=="left")
                {
                    x+=line_length;
                }
                double y=finishline_pose[1]-0.5;
                double theta=starting_pose[2];
                ok_go=go_to(Vector({x,y,theta}),false);
            }

            if (ok_go)
            {
                Time::delay(getPeriod());
                cmd.clear();
                rep.clear();
                cmd.addString("is_navigating");
                if (navigationPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()!=ok)
                    {
                        yInfo()<<"Reached finish line";
                        ok_go=false;
                        state=State::starting;
                    }
                }
            }
        }

        if (state==State::starting)
        {
            Bottle cmd,rep;
            string part=which_part();
            point(pointing_finish,part,true);
            speak("explain-line",true);
            point(pointing_home,part,false);
            speak("explain-end",true);
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
                    cmd.addString(KeyPointTag::knee_left);
                    if (attentionPort.write(cmd,rep))
                    {
                        speak("ready",true);
                        speak("go",false);
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
                                yInfo()<<"Start!";
                            }
                        }
                    }
                }
            }
        }

        question_manager->setTime(Time::now()-tstart);
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
                    speak("line-crossed",true);
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
                            speak("not-crossed",false);
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
                    state=State::finished;
                    t=Time::now()-tstart;
                    yInfo()<<"Stop!";
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
                            t=Time::now()-tstart;
                            yInfo()<<"Not passed in"<<t<<"seconds";
                        }
                    }
                }
            }
        }

        if (state==State::finished)
        {
            yInfo()<<"Test finished in"<<t<<"seconds";
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
    string which_part()
    {
        yarp::sig::Matrix R=axis2dcm(finishline_pose.subVector(3,6));
        yarp::sig::Vector u=R.subcol(0,0,2);
        yarp::sig::Vector p0=finishline_pose.subVector(0,2);
        yarp::sig::Vector p1=p0+line_length*u;
        string part="";
        Bottle cmd,rep;
        cmd.addString("get_state");
        if (navigationPort.write(cmd,rep))
        {
            Property robotState(rep.get(0).toString().c_str());
            if (Bottle *loc=robotState.find("robot-location").asList())
            {
                double x=loc->get(0).asDouble();
                double line_center=(p0[0]+p1[0])/2;
                if ( (x-line_center)>0.0 )
                {
                    part="left";
                }
                else
                {
                    part="right";
                }
            }
        }

        return part;
    }

    /****************************************************************/
    bool point(const Vector &target, const string &part, const bool wait)
    {
        if(part.empty())
        {
            return false;
        }

        //if a question was received, we wait until an answer is given, before pointing
        bool received_question=question_manager->gotQuestion();
        if (received_question && wait)
        {
            bool can_point=false;
            yInfo()<<"Replying to question first";
            while (true)
            {
                can_point=question_manager->hasReplied();
                if (can_point)
                {
                    break;
                }
            }
        }

        RpcClient *tmpPort;
        if (part=="left")
        {
            tmpPort=&leftarmPort;
        }
        if (part=="right")
        {
            tmpPort=&rightarmPort;
        }

        Bottle cmd,rep;
        cmd.addString("ctpq");
        cmd.addString("time");
        cmd.addDouble(pointing_time);
        cmd.addString("off");
        cmd.addInt(0);
        cmd.addString("pos");
        cmd.addList().read(target);
        yDebug()<<cmd.toString();
        if (tmpPort->write(cmd,rep))
        {
            if (rep.get(0).asBool()==true)
            {
                yInfo()<<"Moving"<<part<<"arm";
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    Property opcRead(const string &t)
    {
        Property prop;
        if (opcPort.getInputCount()>0)
        {
            if (Bottle* b=opcPort.read(true))
            {
                if (!b->get(1).isString())
                {
                    for (int i=1; i<b->size(); i++)
                    {
                        Property temp;
                        temp.fromString(b->get(i).asList()->toString());
                        if (temp.check(t))
                        {
                            prop=temp;
                        }
                    }
                }
            }
        }
        return prop;
    }

    /****************************************************************/
    bool hasLine(Property &prop)
    {
        prop=opcRead("finish-line");
        if (!prop.isNull())
        {
            return true;
        }

        return false;
    }

    /****************************************************************/
    bool getWorld(const Property &prop)
    {
        if (opcPort.getInputCount()>0)
        {
            Bottle *line=prop.find("finish-line").asList();
            if (Bottle *lp_bottle=line->find("pose_world").asList())
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
                    if (Bottle *lp_length=line->find("size").asList())
                    {
                        if (lp_length->size()>=2)
                        {
                            line_length=lp_length->get(0).asDouble();
                            yInfo()<<"with length"<<line_length;
                            yInfo()<<"World configured";
                            world_configured=true;
                            return true;
                        }
                    }
                }
            }
        }

        yError()<<"Could not configure world";
        return false;
    }

    /****************************************************************/
    string findLocked()
    {
        string t("");
        Property prop=opcRead("tag");
        if (!prop.isNull())
        {
            t=prop.find("tag").asString();
            if (t.find("-locked")!=string::npos)
            {
                yInfo()<<"Found locked skeleton"<<t;
            }
        }

        return t;
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
        question_manager->interrupt();
        question_manager->close();
        delete question_manager;
        if (detect_hand_up)
        {
            hand_manager->stop();
            delete hand_manager;
        }
        trigger_manager->stop();
        delete trigger_manager;
        analyzerPort.close();
        speechRpcPort.close();
        attentionPort.close();
        navigationPort.close();
        leftarmPort.close();
        rightarmPort.close();
        speechStreamPort.close();
        opcPort.close();
        cmdPort.close();
        lockerPort.close();
        triggerPort.close();
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


