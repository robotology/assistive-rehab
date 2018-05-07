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
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>

#include <yarp/os/all.h>
#include <yarp/math/Rand.h>

#include "AssistiveRehab/skeleton.h"

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
class Interaction : public RFModule
{
    const int ok=Vocab::encode("ok");
    const int fail=Vocab::encode("fail");

    enum class State { idle, seek, follow, engaged, assess } state;
    double period;
    string tag;
    int reinforce_engage_cnt;
    double T,t0,t1;

    unordered_map<string,unordered_set<string>> history;
    unordered_map<string,string> speak_map;
    vector<double> assess_values;

    RpcClient attentionPort;
    RpcClient analyzerPort;
    BufferedPort<Bottle> speechStreamPort;
    RpcClient speechRpcPort;
    bool interrupting;

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

        while (wait && (speechRpcPort.getOutputCount()>0))
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
            if (interrupting)
            {
                break;
            }
        }
        return (it!=end(speak_map));
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
    bool disengage()
    {
        state=State::idle;
        Bottle cmd,rep;
        cmd.addString("set_auto");
        if (attentionPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==ok)
            {
                return true;
            }
        }
        t0=Time::now();
        return false;
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
            unordered_set<string> s1,diff;
            for (int i=0; i<metrics.size(); i++)
            {
                s1.insert(metrics.get(i).asString());
            }
            auto &s2=it1->second;
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
        if (mean<0.33)
        {
            grade="low";
        }
        else if (mean<0.66)
        {
            grade="medium";
        }
        else
        {
            grade="high";
        }
        speak("assess-"+phase+"-"+grade,false);
        return true;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        period=rf.check("period",Value(0.1)).asDouble();
        string speak_file=rf.check("speak-file",Value("speak-it")).asString();

        if (!load_speak(rf.getContext(),speak_file))
        {
            string msg="Unable to locate file";
            msg+="\""+speak_file+"\"";
            yError()<<msg;
            return false;
        }

        attentionPort.open("/interactionManager/attention:rpc");
        analyzerPort.open("/interactionManager/analyzer:rpc");
        speechStreamPort.open("/interactionManager/speech:o");
        speechRpcPort.open("/interactionManager/speech:rpc");

        Rand::init();
        state=State::idle;
        interrupting=false;
        t0=Time::now();
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
        if ((attentionPort.getOutputCount()==0) || (analyzerPort.getOutputCount()==0) ||
            (speechStreamPort.getOutputCount()==0) || (speechRpcPort.getOutputCount()==0))
        {
            yInfo()<<"not connected";
            return true;
        }

        string follow_tag;
        {
            Bottle cmd,rep;
            cmd.addString("is_following");
            if (attentionPort.write(cmd,rep))
            {
                follow_tag=rep.get(0).asString();
            }
        }

        if (state>=State::follow)
        {
            if (follow_tag!=tag)
            {
                if (state==State::assess)
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
            if (!follow_tag.empty())
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
                        T=rep.get(0).asDouble();
                        if (T>0.0)
                        {
                            cmd.clear();
                            cmd.addString("selectSkel");
                            cmd.addString(tag);
                            if (analyzerPort.write(cmd,rep))
                            {
                                if (rep.get(0).asVocab()==ok)
                                {
                                    speak("explain",true);

                                    vector<SpeechParam> p;
                                    p.push_back(SpeechParam(T));
                                    speak("duration",true,p);

                                    cmd.clear();
                                    cmd.addString("start");
                                    if (analyzerPort.write(cmd,rep))
                                    {
                                        if (rep.get(0).asVocab()==ok)
                                        {
                                            speak("ready",true);
                                            Time::delay(3.0);
                                            speak("start",true);
                                            history[tag].insert(metric);

                                            state=State::assess;
                                            assess_values.clear();
                                            t0=Time::now();
                                            t1=t0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (state!=State::assess)
            {
                speak("ouch",true);
                disengage();
            }
        }

        if (state==State::assess)
        {
            double t_0=(Time::now()-t0)/T;
            double t_1=(Time::now()-t1)/T;
            if (t_0<=1.0)
            {
                Bottle cmd,rep;
                cmd.addString("getQuality");
                if (analyzerPort.write(cmd,rep))
                {
                    assess_values.push_back(rep.get(0).asDouble());
                }
                if (t_1>0.55)
                {
                    assess("intermediate");
                    t1=Time::now();
                }
            }
            else
            {
                Bottle cmd,rep;
                cmd.addString("stop");
                analyzerPort.write(cmd,rep);

                speak("end",true);
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
        attentionPort.close();
        analyzerPort.close();
        speechStreamPort.close();
        speechRpcPort.close();
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


