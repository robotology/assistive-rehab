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
#include <unordered_map>
#include <string>
#include <sstream>

#include <yarp/os/all.h>
#include <yarp/math/Rand.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::math;


/****************************************************************/
class Interaction : public RFModule
{
    enum class State { seek, follow, show, engaged } state;
    double period;
    string tag;
    string metric;
    double t0;

    unordered_map<string,string> speak_map;

    RpcClient attentionPort;
    RpcClient analyzerPort;
    BufferedPort<Bottle> speechPort;

    /****************************************************************/
    void speak(const string &value)
    {
        Bottle &payload=speechPort.prepare();
        payload.clear();

        payload.addString(speak_map[value]);
        speechPort.writeStrict();
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
                yError()<<"Unable to find key \"kaye\" and/or \"value\"";
                return false;
            }
            string key=bSection.find("key").asString();
            string value=bSection.find("value").asString();
            speak_map[key]=value;
        }

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
        speechPort.open("/interactionManager/speech:o");

        state=State::seek;
        Rand::init();
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
        if (attentionPort.getOutputCount()==0)
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

        if (state==State::seek)
        {
            if (!follow_tag.empty())
            {
                tag=follow_tag;
                Bottle cmd,rep;
                cmd.addString("look");
                cmd.addString(tag);
                if (attentionPort.write(cmd,rep))
                {
                    speak("invite");
                    speak("engage");
                    state=State::follow;
                    t0=Time::now();
                }                
            }
        }
        else if (state==State::follow)
        {
            if (follow_tag!=tag)
            {
                speak("disengaged");
                state=State::seek;
            }
            else
            {
                Bottle cmd,rep;
                cmd.addString("is_with_raised_hand");
                cmd.addString(tag);
                if (attentionPort.write(cmd,rep))
                {
                    if (rep.get(0).asBool())
                    {
                        speak("accepted");
                        state=State::show;
                    }
                    else if (Time::now()-t0>5.0)
                    {
                        speak("reinforce-engage");
                        t0=Time::now();
                    }
                }
            }
        }
        else if (state==State::show)
        {
            speak("show");
            state=State::engaged;
        }
        else if (state==State::engaged)
        {
            Bottle cmd,rep;
            cmd.addString("listMetrics");
            if (analyzerPort.write(cmd,rep))
            {
                if (rep.size()>0)
                {
                    metric=rep.get((int)(Rand::scalar(0,1)*rep.size())).asString();
                    cmd.clear();
                    cmd.addString("loadMetric");
                    cmd.addString(metric);
//                  if (analyzerPort.write(cmd,rep))
//                  {
//                  }
                }
            }
        }

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        attentionPort.close();
        analyzerPort.close();
        speechPort.close();
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


