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
#include <math.h>
#include <map>

#include <yarp/os/all.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace assistive_rehab;

/****************************************************************/
class SpeechParam
{
    ostringstream ss;
public:
    SpeechParam(const string &s) { ss<<s; }
    string get() const { return ss.str(); }
};

/********************************************************/
class Feedback : public BufferedPort<Bottle>
{
    string moduleName;
    BufferedPort<Bottle> speechPort;

    double var_thresh,skwns_thresh;
    int range_freq;
    map<string,string> joint2bodypart;
    map<string,string> speak_map;

public:

    /********************************************************/
    Feedback(const string &moduleName_, const string &context, const string &speak_file,
             const double &var_thresh_, const double &skwns_thresh_, const int range_freq_)
    {
        moduleName = moduleName_;
        var_thresh = var_thresh_;
        skwns_thresh = skwns_thresh_;
        range_freq = range_freq_;

        if(!load_speak(context,speak_file))
        {
            string msg="Unable to locate file";
            msg+="\""+speak_file+"\"";
            yError()<<msg;
        }

        //left arm
        joint2bodypart[KeyPointTag::shoulder_left]="armLeft";
        joint2bodypart[KeyPointTag::elbow_left]="armLeft";
        joint2bodypart[KeyPointTag::hand_left]="armLeft";

        //right arm
        joint2bodypart[KeyPointTag::shoulder_right]="armRight";
        joint2bodypart[KeyPointTag::elbow_right]="armRight";
        joint2bodypart[KeyPointTag::hand_right]="armRight";

        //left leg
        joint2bodypart[KeyPointTag::hip_left]="legLeft";
        joint2bodypart[KeyPointTag::knee_left]="legLeft";
        joint2bodypart[KeyPointTag::ankle_left]="legLeft";

        //right leg
        joint2bodypart[KeyPointTag::hip_right]="legRight";
        joint2bodypart[KeyPointTag::knee_right]="legRight";
        joint2bodypart[KeyPointTag::ankle_right]="legRight";

        //torso
        joint2bodypart[KeyPointTag::shoulder_center]="torso";
        joint2bodypart[KeyPointTag::hip_center]="torso";

        //head
        joint2bodypart[KeyPointTag::head]="head";
    }

    /********************************************************/
    ~Feedback()
    {
    }

    /********************************************************/
    bool open()
    {
        this->useCallback();
        BufferedPort<Bottle>::open("/" + moduleName + "/feedback:i");
        speechPort.open("/" + moduleName + "/speech:o");

        return true;
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

    /********************************************************/
    void onRead(Bottle &data)
    {
        if(Bottle *feedb = data.get(0).asList())
        {
            map<string,vector<double>> bodypart2feed;
            bodypart2feed.clear();
            for(size_t i=0; i<feedb->size(); i++)
            {
                //we cluster joints into body parts
                //and associate the worst feedback we get from single joints
                if(Bottle *joint_list = feedb->get(i).asList())
                {
                    string bp = joint2bodypart.at(joint_list->get(0).asString());
//                    bodypart2feed[bp].resize(8);
                    if(Bottle *feed_pos_x = joint_list->get(1).asList())
                    {
                        if(bodypart2feed[bp].size() > 0)
                        {
                            //we consider the worst case: highest skewness and/or variance
                            bool highest_skwns = fabs(feed_pos_x->get(2).asDouble()) > fabs(bodypart2feed[bp][1]);
                            bool highest_var_skwns = (feed_pos_x->get(1).asDouble() > bodypart2feed[bp][0] &&
                                    fabs(feed_pos_x->get(2).asDouble()) > fabs(bodypart2feed[bp][1]));
                            if( highest_var_skwns || highest_skwns )
                            {
                                bodypart2feed[bp][0] = feed_pos_x->get(1).asDouble();
                                bodypart2feed[bp][1] = feed_pos_x->get(2).asDouble();
                            }
                        }
                        else
                        {
                            bodypart2feed[bp].push_back(feed_pos_x->get(1).asDouble());
                            bodypart2feed[bp].push_back(feed_pos_x->get(2).asDouble());
                        }
                    }
                    if(Bottle *feed_pos_y = joint_list->get(2).asList())
                    {
                        if(bodypart2feed[bp].size() > 2)
                        {
                            //we consider the worst case: highest skewness and/or variance
                            bool highest_skwns = fabs(feed_pos_y->get(2).asDouble()) > fabs(bodypart2feed[bp][3]);
                            bool highest_var_skwns = (feed_pos_y->get(1).asDouble() > bodypart2feed[bp][2] &&
                                    fabs(feed_pos_y->get(2).asDouble()) > fabs(bodypart2feed[bp][3]));
                            if( highest_var_skwns || highest_skwns )
                            {
                                bodypart2feed[bp][2] = feed_pos_y->get(1).asDouble();
                                bodypart2feed[bp][3] = feed_pos_y->get(2).asDouble();
                            }
                        }
                        else
                        {
                            bodypart2feed[bp].push_back(feed_pos_y->get(1).asDouble());
                            bodypart2feed[bp].push_back(feed_pos_y->get(2).asDouble());
                        }
                    }
                    if(Bottle *feed_pos_z = joint_list->get(3).asList())
                    {
                        if(bodypart2feed[bp].size() > 4)
                        {
                            //we consider the worst case: highest skewness and/or variance
                            bool highest_skwns = fabs(feed_pos_z->get(2).asDouble()) > fabs(bodypart2feed[bp][5]);
                            bool highest_var_skwns = (feed_pos_z->get(1).asDouble() > bodypart2feed[bp][4] &&
                                    fabs(feed_pos_z->get(2).asDouble()) > fabs(bodypart2feed[bp][5]));
                            if( highest_var_skwns || highest_skwns )
                            {
                                bodypart2feed[bp][4] = feed_pos_z->get(1).asDouble();
                                bodypart2feed[bp][5] = feed_pos_z->get(2).asDouble();
                            }
                        }
                        else
                        {
                            bodypart2feed[bp].push_back(feed_pos_z->get(1).asDouble());
                            bodypart2feed[bp].push_back(feed_pos_z->get(2).asDouble());
                        }
                    }
                    if(Bottle *feed_speed = joint_list->get(4).asList())
                    {
                        if(bodypart2feed[bp].size() > 6)
                        {
                            //we consider the worst case: highest difference in frequency
                            bool highest_deltaf = fabs(feed_speed->get(1).asInt()-feed_speed->get(2).asInt()) >
                                    fabs(bodypart2feed[bp][6]-bodypart2feed[bp][7]);
                            if(highest_deltaf)
                            {
                                bodypart2feed[bp][6] = feed_speed->get(1).asInt();
                                bodypart2feed[bp][7] = feed_speed->get(2).asInt();
                            }
                        }
                        else
                        {
                            bodypart2feed[bp].push_back(feed_speed->get(1).asInt());
                            bodypart2feed[bp].push_back(feed_speed->get(2).asInt());
                        }
                    }
                }
            }
            print(bodypart2feed);

            //we use this structure for providing a feedback
            assess(bodypart2feed);
        }

    }

    /********************************************************/
    void print(const map<string,vector<double>> &bodypart2feed)
    {
        for(auto &it : bodypart2feed)
        {
            string bodypart = it.first;
            double var_x = it.second[0];
            double skwns_x = it.second[1];
            double var_y = it.second[2];
            double skwns_y = it.second[3];
            double var_z = it.second[4];
            double skwns_z = it.second[5];
            int ftemplate = it.second[6];
            int fcandidate = it.second[7];

            cout << bodypart
                 << " ( x " << var_x << " " << skwns_x << " )"
                 << " ( y " << var_y << " " << skwns_y << " )"
                 << " ( z " << var_z << " " << skwns_z << " )"
                 << " ( freq: " << ftemplate << " " << fcandidate << " )"
                 << endl;
        }
        cout << endl;
    }

    /********************************************************/
    bool assess(const map<string,vector<double>> &bodypart2feed)
    {
        vector<string> err_static;
        vector<pair<string,string>> err_pos,err_speed;
        err_static.clear();
        err_pos.clear();
        err_speed.clear();
        for(auto &it : bodypart2feed)
        {
            string bodypart = it.first;
            int ftemplate = it.second[6];
            int fcandidate = it.second[7];
            if(ftemplate == 0)
            {
                //error static joints
                if(fcandidate != 0)
                {
                    err_static.push_back(bodypart);
                }
            }
            else
            {
                //error position
                double var_x = it.second[0];
                double skwns_x = it.second[1];
                if(var_x < var_thresh && skwns_x > 0.0)
                {
                    err_pos.push_back(make_pair(bodypart,"a sinistra"));
                }
                else if(var_x < var_thresh && skwns_x < 0.0)
                {
                    err_pos.push_back(make_pair(bodypart,"a destra"));
                }
                else if(var_x > var_thresh && fabs(skwns_x) < skwns_thresh)
                {
                    err_pos.push_back(make_pair(bodypart,"a sinistra e destra"));
                }

                double var_y = it.second[2];
                double skwns_y = it.second[3];
                if(var_y < var_thresh && skwns_y > 0.0)
                {
                    err_pos.push_back(make_pair(bodypart,"su"));
                }
                else if(var_y < var_thresh && skwns_y < 0.0)
                {
                    err_pos.push_back(make_pair(bodypart,"giu'"));
                }
                else if(var_y > var_thresh && fabs(skwns_y) < skwns_thresh)
                {
                    err_pos.push_back(make_pair(bodypart,"su e giu'"));
                }

                double var_z = it.second[4];
                double skwns_z = it.second[5];
                if(var_z < var_thresh && skwns_z > 0.0)
                {
                    err_pos.push_back(make_pair(bodypart,"avanti"));
                }
                else if(var_z < var_thresh && skwns_z < 0.0)
                {
                    err_pos.push_back(make_pair(bodypart,"dietro"));
                }
                else if(var_z > var_thresh && fabs(skwns_z) < skwns_thresh)
                {
                    err_pos.push_back(make_pair(bodypart,"avanti e dietro"));
                }

                //error speed
                int df = fcandidate-ftemplate;
                if(df > range_freq)
                {
                    err_speed.push_back(make_pair(bodypart,"velocemente"));
                }
                else if(df < -range_freq)
                {
                    err_speed.push_back(make_pair(bodypart,"lentamente"));
                }
            }
        }

        //first check to be passed: all static body parts are not moving
        if(err_static.size() > 0)
        {
            vector<SpeechParam> params;
            for(int i=0; i<err_static.size(); i++)
            {
                params.clear();
                params.push_back(err_static[i]);
                speak("static",params);
            }
            return true;
        }

        //second check to be passed: no error in position
        if(err_pos.size() > 0)
        {
            vector<SpeechParam> params;
            for(int i=0; i<err_pos.size(); i++)
            {
                params.clear();
                params.push_back(err_pos[i].first);
                params.push_back(err_pos[i].second);
                speak("position",params);
            }
            return true;
        }

        //third check to be passed: no error in speed
        if(err_speed.size() > 0)
        {
            vector<SpeechParam> params;
            for(int i=0; i<err_speed.size(); i++)
            {
                params.clear();
                params.push_back(err_speed[i].first);
                params.push_back(err_speed[i].second);
                speak("speed",params);
            }
            return true;
        }

        speak("perfect");
        return true;

    }
    /********************************************************/
    void speak(const string &key, const vector<SpeechParam> &p=vector<SpeechParam>())
    {
        string value = speak_map.at(key);
        if(p.size() > 0)
        {
            size_t pos1 = value.find("%");
            string value_extra = value.substr(pos1+1,value.length());
            string f1 = p[0].get();
            value.replace(pos1,f1.length(),f1);
            value.erase(pos1+f1.length(),value.length());
            value += value_extra;

            if(p.size() > 1)
            {
                size_t pos2 = value.find("#");
                string f2 = p[1].get();
                value_extra.clear();
                value_extra = value.substr(pos2+1,value.length());
                value.replace(pos2,f2.length(),f2);
                value += value_extra;
            }
        }

        yInfo() << value;

        Bottle &out = speechPort.prepare();
        out.clear();
        out.addString(value);
        speechPort.writeStrict();

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
    }


};

/********************************************************/
class Module : public RFModule
{
    Feedback *feedback;

    double period,var_thresh,skwns_thresh;
    int range_freq;

public:

    /********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string moduleName = rf.check("name",Value("feedbackSynthetizer")).asString();
        setName(moduleName.c_str());

        string speak_file=rf.check("speak-file",Value("speak-it")).asString();
        period = rf.check("period",Value(0.1)).asDouble();
        var_thresh = rf.check("var_thresh",Value(0.5)).asDouble();
        skwns_thresh = rf.check("skwns_thresh",Value(0.5)).asDouble();
        range_freq = rf.check("range_freq",Value(2)).asInt();

        feedback = new Feedback(moduleName,rf.getContext(),speak_file,var_thresh,skwns_thresh,range_freq);
        feedback->open();

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
        feedback->interrupt();
        feedback->close();
        delete feedback;
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
