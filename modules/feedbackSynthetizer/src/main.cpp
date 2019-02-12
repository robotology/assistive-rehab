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
#include <cmath>
#include <map>

#include <yarp/os/all.h>
#include <yarp/math/Math.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

namespace BodyPartTag
{
const string arm_left = "armLeft";
const string arm_right = "armRight";
const string leg_left = "legLeft";
const string leg_right = "legRight";
const string head = "head";
const string torso = "torso";
}

/****************************************************************/
class SpeechParam
{
    ostringstream ss;
public:
    SpeechParam();
    SpeechParam(const SpeechParam &sp) { ss<<sp.get();}
    SpeechParam(const string &s) { ss<<s; }
    string get() const { return ss.str(); }
};

/********************************************************/
class Synthetizer : public BufferedPort<Bottle>
{
    string moduleName;
    BufferedPort<Bottle> speechPort;
    BufferedPort<Bottle> scorePort;

    map<string,string> bodypart2verbal;
    map<string,pair<vector<string>,vector<string>>> speak_map;
    map<string,int> priority_map;
    map<string,string> joint2bodypart;

    int speak_length;
    string conj;
    int maxpriority;

public:

    /********************************************************/
    Synthetizer(const string &moduleName_, const string &context, const string &speak_file,
                const int speak_length_)
    {
        moduleName = moduleName_;
        speak_length = speak_length_;

        string armLeft,armRight,legLeft,legRight,torso,head;
        if(!load_speak(context,speak_file,armLeft,armRight,legLeft,legRight,torso,head))
        {
            string msg="Unable to locate file";
            msg+="\""+speak_file+"\"";
            yError()<<msg;
        }

        //left arm
        joint2bodypart[KeyPointTag::shoulder_left]=BodyPartTag::arm_left;
        joint2bodypart[KeyPointTag::elbow_left]=BodyPartTag::arm_left;
        joint2bodypart[KeyPointTag::hand_left]=BodyPartTag::arm_left;

        //right arm
        joint2bodypart[KeyPointTag::shoulder_right]=BodyPartTag::arm_right;
        joint2bodypart[KeyPointTag::elbow_right]=BodyPartTag::arm_right;
        joint2bodypart[KeyPointTag::hand_right]=BodyPartTag::arm_right;

        //left leg
        joint2bodypart[KeyPointTag::hip_left]=BodyPartTag::leg_left;
        joint2bodypart[KeyPointTag::knee_left]=BodyPartTag::leg_left;
        joint2bodypart[KeyPointTag::ankle_left]=BodyPartTag::leg_left;

        //right leg
        joint2bodypart[KeyPointTag::hip_right]=BodyPartTag::leg_right;
        joint2bodypart[KeyPointTag::knee_right]=BodyPartTag::leg_right;
        joint2bodypart[KeyPointTag::ankle_right]=BodyPartTag::leg_right;

        //torso
        joint2bodypart[KeyPointTag::shoulder_center]=BodyPartTag::torso;
        joint2bodypart[KeyPointTag::hip_center]=BodyPartTag::torso;

        //head
        joint2bodypart[KeyPointTag::head]=BodyPartTag::head;

        //translate body part to verbal string
        bodypart2verbal[BodyPartTag::arm_left]=armLeft;
        bodypart2verbal[BodyPartTag::arm_right]=armRight;
        bodypart2verbal[BodyPartTag::leg_left]=legLeft;
        bodypart2verbal[BodyPartTag::leg_right]=legRight;
        bodypart2verbal[BodyPartTag::torso]=torso;
        bodypart2verbal[BodyPartTag::head]=head;

        priority_map["speed"]=0;
        priority_map["position-rom"]=1;
        priority_map["perfect"]=2;
        priority_map["position-ep"]=-1;
        maxpriority=0;
    }

    /********************************************************/
    ~Synthetizer()
    {
    }

    /********************************************************/
    bool open()
    {
        this->useCallback();
        BufferedPort<Bottle>::open("/" + moduleName + "/dtw:i");
        speechPort.open("/" + moduleName + "/speech:o");
        scorePort.open("/" + moduleName + "/score:o");

        return true;
    }

    /****************************************************************/
    bool load_speak(const string &context, const string &speak_file,
                    string &armLeft, string &armRight, string &legLeft, string &legRight, string &torso, string &head)
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
        if (!bGroup.check("conjunction"))
        {
            yError()<<"Unable to find key \"conjunction\"";
            return false;
        }

        conj = bGroup.find("conjunction").asString();
        Bottle &bBody=rf_speak.findGroup("body");
        if (bBody.isNull())
        {
            yError()<<"Unable to find group \"body\"";
            return false;
        }
        armLeft = bBody.find("armLeft").asString();
        armRight = bBody.find("armRight").asString();
        legLeft = bBody.find("legLeft").asString();
        legRight = bBody.find("legRight").asString();
        torso = bBody.find("torso").asString();
        head = bBody.find("head").asString();

        Bottle &bSpeed=rf_speak.findGroup("speed");
        if (bSpeed.isNull() || !bSpeed.check("key")
                || !bSpeed.check("value") || !bSpeed.check("feedback"))
        {
            yError()<<"Unable to find group \"speed\" and/or find key "
                      "\"key\" and/or \"value\" and/or \"feedback\"";
            return false;
        }
        string key = bSpeed.find("key").asString();
        Bottle *bValue = bSpeed.find("value").asList();
        Bottle *bFeedback = bSpeed.find("feedback").asList();
        vector<string> val,feedb;
        val.clear();
        for(int i=0; i<bValue->size(); i++)
        {
            val.push_back(bValue->get(i).asString());
        }
        feedb.clear();
        for(int i=0; i<bFeedback->size(); i++)
        {
            feedb.push_back(bFeedback->get(i).asString());
        }
        speak_map[key] = make_pair(val,feedb);

        Bottle &bRom=rf_speak.findGroup("position-rom");
        if (bRom.isNull() || !bRom.check("key") || !bRom.check("value") || !bRom.check("feedback"))
        {
            yError()<<"Unable to find group \"position-rom\" and/or find key "
                      "\"key\" and/or \"value\" and/or \"feedback\"";
            return false;
        }
        key.clear();
        bValue->clear();
        bFeedback->clear();
        key = bRom.find("key").asString();
        bValue = bRom.find("value").asList();
        bFeedback = bRom.find("feedback").asList();
        val.clear();
        for(int i=0; i<bValue->size(); i++)
        {
            val.push_back(bValue->get(i).asString());
        }
        feedb.clear();
        for(int i=0; i<bFeedback->size(); i++)
        {
            feedb.push_back(bFeedback->get(i).asString());
        }
        speak_map[key] = make_pair(val,feedb);

        Bottle &bEp=rf_speak.findGroup("position-ep");
        if (bEp.isNull() || !bEp.check("key") || !bEp.check("value") || !bEp.check("feedback"))
        {
            yError()<<"Unable to find group \"position-ep\" and/or find key "
                      "\"key\" and/or \"value\" and/or \"feedback\"";
            return false;
        }
        key.clear();
        bValue->clear();
        bFeedback->clear();
        key = bEp.find("key").asString();
        bValue = bEp.find("value").asList();
        bFeedback = bEp.find("feedback").asList();
        val.clear();
        for(int i=0; i<bValue->size(); i++)
        {
            val.push_back(bValue->get(i).asString());
        }
        feedb.clear();
        for(int i=0; i<bFeedback->size(); i++)
        {
            feedb.push_back(bFeedback->get(i).asString());
        }
        speak_map[key] = make_pair(val,feedb);

        Bottle &bPerfect=rf_speak.findGroup("perfect");
        if (bPerfect.isNull() || !bPerfect.check("key")
                || !bPerfect.check("value") || !bPerfect.check("feedback"))
        {
            yError()<<"Unable to find group \"perfect\" and/or find key "
                      "\"key\" and/or \"value\" and/or \"feedback\"";
            return false;
        }
        key.clear();
        bValue->clear();
        bFeedback->clear();
        key = bPerfect.find("key").asString();
        bValue = bPerfect.find("value").asList();
        bFeedback = bPerfect.find("feedback").asList();
        val.clear();
        for(int i=0; i<bValue->size(); i++)
        {
            val.push_back(bValue->get(i).asString());
        }
        feedb.clear();
        for(int i=0; i<bFeedback->size(); i++)
        {
            feedb.push_back(bFeedback->get(i).asString());
        }
        speak_map[key] = make_pair(val,feedb);

        Bottle &bWrong=rf_speak.findGroup("wrong");
        if (bWrong.isNull() || !bWrong.check("key")
                || !bWrong.check("value") || !bWrong.check("feedback"))
        {
            yError()<<"Unable to find group \"wrong\" and/or find key "
                      "\"key\" and/or \"value\" and/or \"feedback\"";
            return false;
        }
        key.clear();
        bValue->clear();
        bFeedback->clear();
        key = bWrong.find("key").asString();
        bValue = bWrong.find("value").asList();
        bFeedback = bWrong.find("feedback").asList();
        val.clear();
        for(int i=0; i<bValue->size(); i++)
        {
            val.push_back(bValue->get(i).asString());
        }
        feedb.clear();
        for(int i=0; i<bFeedback->size(); i++)
        {
            feedb.push_back(bFeedback->get(i).asString());
        }
        speak_map[key] = make_pair(val,feedb);

        Bottle &bStatic=rf_speak.findGroup("static");
        if (bStatic.isNull() || !bStatic.check("key")
                || !bStatic.check("value") || !bStatic.check("feedback"))
        {
            yError()<<"Unable to find group \"static\" and/or find key "
                      "\"key\" and/or \"value\" and/or \"feedback\"";
            return false;
        }
        key.clear();
        bValue->clear();
        bFeedback->clear();
        key = bStatic.find("key").asString();
        bValue = bStatic.find("value").asList();
        bFeedback = bStatic.find("feedback").asList();
        val.clear();
        for(int i=0; i<bValue->size(); i++)
        {
            val.push_back(bValue->get(i).asString());
        }
        feedb.clear();
        for(int i=0; i<bFeedback->size(); i++)
        {
            feedb.push_back(bFeedback->get(i).asString());
        }
        speak_map[key] = make_pair(val,feedb);

        return true;
    }
    
    /********************************************************/
    void onRead(Bottle &data)
    {
        double score;
        if(Bottle *feedb = data.get(0).asList())
        {
            vector<SpeechParam> params;
            vector<string> speak_buffer;
            int priority=2;
            yInfo() << feedb->toString();
            for(size_t j=0; j<feedb->size(); j++)
            {
                Bottle *f=feedb->get(j).asList();
                if(f->size()>1)
                {
                    string joint=f->get(0).asString();
                    string key=f->get(1).asString();
                    if(f->size()>2)
                    {
                        params.clear();
                        params.push_back(joint);
                        speak_buffer.clear();
                        string val=f->get(2).asString();
                        for(size_t j=0; j<speak_map[key].first.size(); j++)
                        {
                            if(speak_map[key].first[j]==val)
                                speak_buffer.push_back(speak_map[key].second[j]);
                        }
                    }
                    else
                    {
                        speak_buffer.clear();
                        for(size_t j=0; j<speak_map[key].second.size(); j++)
                        {
                            speak_buffer.push_back(speak_map[key].second[j]);
                        }
                    }
                    if(priority_map[key]<priority)
                        priority=priority_map[key];
                    if(priority==maxpriority)
                        break;
                }
                else
                {
                    //wrong exercise or static
                    priority=-1;
                    string key=f->get(0).asString();
                    params.clear();
                    speak_buffer.clear();
                    for(size_t j=0; j<speak_map[key].second.size(); j++)
                    {
                        speak_buffer.push_back(speak_map[key].second[j]);
                    }
                }
            }
            speak(speak_buffer,params);

            switch (priority)
            {
            case -1: //wrong exercise or static
                score=0.0;
                break;
            case 0: //error in speed
                score = 0.5;
                break;
            case 1: //error in position
                score = 0.5;
                break;
            case 2: //no error
                score = 1.0;
                break;
            }
            yInfo() << "score:" << score;
        }
        Bottle &outscore = scorePort.prepare();
        outscore.clear();
        outscore.addDouble(score);
        scorePort.write();
    }

    /********************************************************/
    void speak(const vector<string> &buffer, const vector<SpeechParam> &p)
    {
        for(size_t i=0; i<buffer.size(); i++)
        {
            if(i>=speak_length)
                break;

            string value = buffer[i];
            if(p.size() > 0)
            {
                size_t pos1 = value.find("%");
                string value_ex = value.substr(pos1+1,value.length());
                string f1 = bodypart2verbal[joint2bodypart[p[0].get()]];
                value.replace(pos1,f1.length(),f1);
                value.erase(pos1+f1.length(),value.length());
                value += value_ex;
            }
            yWarning() << value;
            Bottle &out = speechPort.prepare();
            out.clear();
            out.addString(value);
            speechPort.writeStrict();
        }
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<Bottle >::interrupt();
    }

    /********************************************************/
    void close()
    {
        BufferedPort<Bottle >::close();
        speechPort.close();
        scorePort.close();
    }


};

/********************************************************/
class Module : public RFModule
{
    Synthetizer *synthetizer;
    double period;

public:

    /********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string moduleName = rf.check("name",Value("feedbackSynthetizer")).asString();
        setName(moduleName.c_str());

        string speak_file=rf.check("speak-file",Value("speak-it")).asString();
        period = rf.check("period",Value(0.1)).asDouble();
        int speak_length = rf.check("speak-length",Value(2)).asInt();

        synthetizer = new Synthetizer(moduleName,rf.getContext(),speak_file,speak_length);
        synthetizer->open();

        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return period;
    }

    /********************************************************/
    bool updateModule()
    {
        return true;
    }

    /********************************************************/
    bool close()
    {
        synthetizer->interrupt();
        synthetizer->close();
        delete synthetizer;
        return true;
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
    rf.setDefaultContext("feedbackSynthetizer");
    rf.setDefaultConfigFile("config-it.ini");
    rf.configure(argc,argv);

    Module module;

    return module.runModule(rf);
}
