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
class BodyFeedback
{
    //levels of the hierarchy:
    //0: static joints moving
    //1: dynamic joints not moving
    //2: error in position
    //3: error in speed
    //4: no error (if no previous level is reached from any of the body parts)

    //FIRST LEVEL OF INTEGRATION
    //from xyz to a level of priority for the single joint
    map<string,pair<int,vector<string>>> xyz2joint;
    map<string,string> joint2bodypart;

    //SECOND LEVEL OF INTEGRATION:
    //this structure associates to each body part a pair that contains:
    //a vector of int: the level of priority and the number of joints with the minimum level of the hierarchy
    //a vector of string: the verbal feedback for that level of the hierarchy
    map<string,pair<int,vector<string>>> bodypart2feedback;
    int maxh,finh;

public:
    /****************************************************************/
    BodyFeedback(const int maxlevel)
    {
        maxh = maxlevel;

        xyz2joint.clear();

        vector<string> init;
        init.clear();
        init.push_back("");
        bodypart2feedback.clear();
        bodypart2feedback[BodyPartTag::arm_left] = make_pair(maxh,init);
        bodypart2feedback[BodyPartTag::arm_right] = make_pair(maxh,init);
        bodypart2feedback[BodyPartTag::leg_left] = make_pair(maxh,init);
        bodypart2feedback[BodyPartTag::leg_right] = make_pair(maxh,init);
        bodypart2feedback[BodyPartTag::torso] = make_pair(maxh,init);
        bodypart2feedback[BodyPartTag::head] = make_pair(maxh,init);

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
    }

    /****************************************************************/
    void createHierarchy(const string &joint, const pair<int,vector<string>> &feed_jnt)
    {
        xyz2joint[joint] = feed_jnt;
    }

    /****************************************************************/
    void updateBodyPart()
    {
        int minh = maxh;
        for(auto &it: xyz2joint)
        {
            string bp = joint2bodypart[it.first];
            vector<string> feedback = it.second.second;
            int h = it.second.first;
            if(h <= bodypart2feedback[bp].first)
                bodypart2feedback[bp] = make_pair(h,feedback);

            if(h < minh)
                minh = h;
        }

        //minimum level
        finh = minh;
    }

    /****************************************************************/
    pair<int,vector<string>> getJointsMinLev(const string &bodypart)
    {
        int count = 0;
        vector<string> jnts;
        jnts.clear();
        for(auto &it: xyz2joint)
        {
            string jnt = it.first;
            int h = it.second.first;
            if(joint2bodypart[jnt] == bodypart && finh == h)
            {
                count++;
                jnts.push_back(jnt);
            }
        }

        return make_pair(count,jnts);
    }

    /****************************************************************/
    int getMinLev() const { return finh; }

    /****************************************************************/
    pair<vector<string>, vector<vector<string>>> getFeedback()
    {
        vector<string> bp;
        vector<vector<string>> finalf;
        bp.clear();
        finalf.clear();
        for(auto &it: bodypart2feedback)
        {
            string bodypart = it.first;
            int h = it.second.first;
            vector<string> feedback = it.second.second;
            if(h == finh)
            {
                pair<int,vector<string>> jnts = getJointsMinLev(bodypart);
                int njnts = jnts.first;
                if(njnts > 1)
                    bp.push_back(bodypart);
                else
                {
                    if(jnts.second.size() > 0)
                    {
                        for(size_t i=0; i<jnts.second.size(); i++)
                            bp.push_back(jnts.second[i]);
                    }
                }
                finalf.push_back(feedback);
            }
        }

        return make_pair(bp,finalf);
    }

    /****************************************************************/
    void print()
    {
        for(auto &it: bodypart2feedback)
        {
            if(it.second.first != 3)
                yInfo() << it.first << it.second.first;
        }
    }

};

/****************************************************************/
class JointFeedback
{
    string j;
    Matrix f;

public:

    /****************************************************************/
    JointFeedback()
    {
        f.resize(3,5);
    }

    /****************************************************************/
    void setName(const string &joint_name)
    {
        j = joint_name;
    }

    /****************************************************************/
    void update(const int component, const double dtw, const double std, const double skwn,
                const double ft, const double fc)
    {
        f[component][0] = dtw;
        f[component][1] = std;
        f[component][2] = skwn;
        f[component][3] = ft;
        f[component][4] = fc;
    }

    /****************************************************************/
    void print()
    {
        cout << j << endl;
        for(int i=0; i<f.rows(); i++)
        {
            if(i%3 == 0)
                cout << "x: ";
            else if(i%3 == 1)
                cout << "y: ";
            else if(i%3 == 2)
                cout << "z: ";

            for(int j=0; j<f.cols(); j++)
            {
                cout << f[i][j] << " ";
            }
            cout <<endl;
        }
    }

    Matrix getFeedbackMatrix() const { return f; }
    string getName() const { return j; }

};

/****************************************************************/
class SpeechParam
{
    ostringstream ss;
public:
    SpeechParam(const string &s) { ss<<s; }
    string get() const { return ss.str(); }
};

/********************************************************/
class Synthetizer : public BufferedPort<Bottle>
{
    string moduleName;
    BufferedPort<Bottle> speechPort;

    double dtw_thresh,sdev_thresh,skwns_thresh;
    int f_static,range_freq;
    map<string,string> bodypart2verbal;
    map<string,pair<string,vector<string>>> speak_map;
    int idtw,ikurt,iskwns,ift,ifc;
    int maxlevel;

public:

    /********************************************************/
    Synthetizer(const string &moduleName_, const string &context, const string &speak_file,
                const double &dtw_thresh_, const double &sdev_thresh_, const int f_static_, const int range_freq_)
    {
        moduleName = moduleName_;
        dtw_thresh = dtw_thresh_;
        sdev_thresh = sdev_thresh_;
        f_static = f_static_;
        range_freq = range_freq_;

        vector<string> body,joint;
        body.clear();
        joint.clear();
        if(!load_speak(context,speak_file,body,joint))
        {
            string msg="Unable to locate file";
            msg+="\""+speak_file+"\"";
            yError()<<msg;
        }

        //translate body part to verbal string
        bodypart2verbal[BodyPartTag::arm_left]=body[0];
        bodypart2verbal[BodyPartTag::arm_right]=body[1];
        bodypart2verbal[BodyPartTag::leg_left]=body[2];
        bodypart2verbal[BodyPartTag::leg_right]=body[3];
        bodypart2verbal[BodyPartTag::torso]=body[4];
        bodypart2verbal[BodyPartTag::head]=body[5];

        bodypart2verbal[KeyPointTag::head]=joint[0];
        bodypart2verbal[KeyPointTag::shoulder_left]=joint[1];
        bodypart2verbal[KeyPointTag::elbow_left]=joint[2];
        bodypart2verbal[KeyPointTag::hand_left]=joint[3];
        bodypart2verbal[KeyPointTag::shoulder_right]=joint[4];
        bodypart2verbal[KeyPointTag::elbow_right]=joint[5];
        bodypart2verbal[KeyPointTag::hand_right]=joint[6];
        bodypart2verbal[KeyPointTag::hip_left]=joint[7];
        bodypart2verbal[KeyPointTag::knee_left]=joint[8];
        bodypart2verbal[KeyPointTag::ankle_left]=joint[9];
        bodypart2verbal[KeyPointTag::hip_right]=joint[10];
        bodypart2verbal[KeyPointTag::knee_right]=joint[11];
        bodypart2verbal[KeyPointTag::ankle_right]=joint[12];
        bodypart2verbal[KeyPointTag::hip_center]=joint[13];
        bodypart2verbal[KeyPointTag::shoulder_center]=joint[13];

        idtw = 1;
        ikurt = 2;
        iskwns = 3;
        ift = 5;
        ifc = 6;

        maxlevel = 4;
    }

    /********************************************************/
    ~Synthetizer()
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
    bool load_speak(const string &context, const string &speak_file, vector<string> &body, vector<string> &joint)
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
        if (!bGroup.check("num-sections") || !bGroup.check("body") || !bGroup.check("joint"))
        {
            yError()<<"Unable to find key \"num-sections\" and/or \"body\" and/or \"joint\"";
            return false;
        }

        int num_sections = bGroup.find("num-sections").asInt();
        Bottle *bBody = bGroup.find("body").asList();
        Bottle *bJoint = bGroup.find("joint").asList();
        for(int i=0; i<bBody->size(); i++)
        {
            body.push_back(bBody->get(i).asString());
        }

        for(int i=0; i<bJoint->size(); i++)
        {
            joint.push_back(bJoint->get(i).asString());
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
            if (!bSection.check("key") || !bSection.check("value") || !bSection.check("feedback"))
            {
                yError()<<"Unable to find key \"key\" and/or \"value\"";
                return false;
            }
            string key = bSection.find("key").asString();
            string value = bSection.find("value").asString();
            Bottle *bFeedback = bSection.find("feedback").asList();
            vector<string> feedb;
            feedb.clear();
            for(int i=0; i<bFeedback->size(); i++)
            {
                feedb.push_back(bFeedback->get(i).asString());
            }

            speak_map[key] = make_pair(value,feedb);
        }

        return true;
    }

    /********************************************************/
    void onRead(Bottle &data)
    {
        if(Bottle *feedb = data.get(0).asList())
        {
            BodyFeedback bf(maxlevel);
            for(size_t i=0; i<feedb->size(); i++)
            {
                //we read feedback from the input port and create the object fj
                //which contains feedback for all the component
                JointFeedback fj;
                if(Bottle *joint_list = feedb->get(i).asList())
                {
                    string joint = joint_list->get(0).asString();
                    fj.setName(joint);
                    if(Bottle *feed_x = joint_list->get(1).asList())
                    {
                        fj.update(0,feed_x->get(idtw).asDouble(),feed_x->get(ikurt).asDouble(),feed_x->get(iskwns).asDouble(),
                                  feed_x->get(ift).asInt(),feed_x->get(ifc).asInt());
                    }
                    if(Bottle *feed_y = joint_list->get(2).asList())
                    {
                        fj.update(1,feed_y->get(idtw).asDouble(),feed_y->get(ikurt).asDouble(),feed_y->get(iskwns).asDouble(),
                                  feed_y->get(ift).asInt(),feed_y->get(ifc).asInt());
                    }
                    if(Bottle *feed_z = joint_list->get(3).asList())
                    {
                        fj.update(2,feed_z->get(idtw).asDouble(),feed_z->get(ikurt).asDouble(),feed_z->get(iskwns).asDouble(),
                                  feed_z->get(ift).asInt(),feed_z->get(ifc).asInt());
                    }

                    //assess how the single joint is moving from its components
                    //the result is a pair with the level of the hierarchy for that joint (int)
                    //and the feedback (vector<string>)
                    pair<int,vector<string>> feed_jnt = assess(fj);
//                    if(feed_jnt.first != 3)
//                    {
                        cout << endl;
                        yInfo() << fj.getName() << feed_jnt.first;
                        for(size_t i=0; i<feed_jnt.second.size(); i++)
                            cout << feed_jnt.second[i] << " ";
                        cout << endl;
                        fj.print();
//                    }

                    bf.createHierarchy(joint,feed_jnt);
                }
            }
            cout << endl;

            bf.updateBodyPart();
            int fin_level = bf.getMinLev();
            bf.print();
            pair<vector<string>,vector<vector<string>>> feedbacklist = bf.getFeedback();
            vector<string> bp = feedbacklist.first;
            vector<vector<string>> finalf = feedbacklist.second;

            //speak
            vector<SpeechParam> params;
            switch (fin_level)
            {
            case 0: //static joints moving
            case 1: //dynamic joints not moving
                for(size_t i=0; i<bp.size(); i++)
                {
                    params.clear();
                    params.push_back(bodypart2verbal[bp[i]]);
                    if(fin_level == 0)
                        speak("static",params);
                    else
                        speak("dynamic",params);
                }
                break;

            case 2: //error in position
                for(size_t i=0; i<bp.size(); i++)
                {
                    params.clear();
                    params.push_back(bodypart2verbal[bp[i]]);
                    for(size_t j=0; j<finalf[i].size(); j++)
                        params.push_back(finalf[i][j]);
                    speak("position",params);
                }
                break;

            case 3: //error in speed
                for(size_t i=0; i<bp.size(); i++)
                {
                    params.clear();
                    params.push_back(bodypart2verbal[bp[i]]);
                    for(size_t j=0; j<finalf[i].size(); j++)
                        params.push_back(finalf[i][j]);
                    speak("speed",params);
                }
                break;

            case 4: //no error
                speak("perfect");
                break;
            }
            cout << endl;
        }

    }

    /********************************************************/
    pair<int,vector<string>> assess(JointFeedback f)
    {
        Matrix feedb = f.getFeedbackMatrix();
        double dx = feedb[0][0];
        double kx = feedb[0][1];
        double sx = feedb[0][2];
        int ftx = feedb[0][3];
        int fcx = feedb[0][4];

        double dy = feedb[1][0];
        double ky = feedb[1][1];
        double sy = feedb[1][2];
        int fty = feedb[1][3];
        int fcy = feedb[1][4];

        double dz = feedb[2][0];
        double kz = feedb[2][1];
        double sz = feedb[2][2];
        int ftz = feedb[2][3];
        int fcz = feedb[2][4];

        /*******************/
        /*   FIRST CHECK   */
        /*******************/
        //static joints in the template must not move
        //dynamic joints in the template must move
        {
            bool ftxy = ftx <= f_static && fty <= f_static;
            bool ftxz = ftx <= f_static && ftz <= f_static;
            bool ftyz = fty <= f_static && ftz <= f_static;

            bool fcxy = fcx > f_static && fcy > f_static;
            bool fcxz = fcx > f_static && fcz > f_static;
            bool fcyz = fcy > f_static && fcz > f_static;

            //we check if the difference is significant
            bool dfx = fabs(ftx-fcx) > range_freq;
            bool dfy = fabs(fty-fcy) > range_freq;
            bool dfz = fabs(ftz-fcz) > range_freq;

            if(ftxy && fcxy)
            {
                if(dfx && dfy)
                {
                    vector<string> out;
                    out.push_back("static");
                    return make_pair(0,out);
                }
            }
            if(ftxz && fcxz)
            {
                if(dfx && dfz)
                {
                    vector<string> out;
                    out.push_back("static");
                    return make_pair(0,out);
                }
            }
            if(ftyz && fcyz)
            {
                if(dfy && dfz)
                {
                    vector<string> out;
                    out.push_back("static");
                    return make_pair(0,out);
                }
            }
        }

        {
            bool ftxy = ftx > f_static && fty > f_static;
            bool ftxz = ftx > f_static && ftz > f_static;
            bool ftyz = fty > f_static && ftz > f_static;

            bool fcxy = fcx <= f_static && fcy <= f_static;
            bool fcxz = fcx <= f_static && fcz <= f_static;
            bool fcyz = fcy <= f_static && fcz <= f_static;

            //we check if the difference is significant
            bool dfx = fabs(ftx-fcx) >= range_freq;
            bool dfy = fabs(fty-fcy) >= range_freq;
            bool dfz = fabs(ftz-fcz) >= range_freq;

            if(ftxy && fcxy)
            {
                if(dfx && dfy)
                {
                    vector<string> out;
                    out.push_back("dynamic");
                    return make_pair(1,out);
                }
            }
            if(ftxz && fcxz)
            {
                if(dfx && dfz)
                {
                    vector<string> out;
                    out.push_back("dynamic");
                    return make_pair(1,out);
                }
            }
            if(ftyz && fcyz)
            {
                if(dfy && dfz)
                {
                    vector<string> out;
                    out.push_back("dynamic");
                    return make_pair(1,out);
                }
            }
        }

        /********************/
        /*   SECOND CHECK   */
        /********************/
        //we check the error in position
        {
            bool dtwx = dx < dtw_thresh;
            bool dtwy = dy < dtw_thresh;
            bool dtwz = dz < dtw_thresh;

            bool errx = kx > sdev_thresh;
            bool erry = ky > sdev_thresh;
            bool errz = kz > sdev_thresh;
            if( errx || erry || erry )
            {
                vector<string> out;
                if(errx)
                {
                    if(sx > 0.0)
                        out.push_back(speak_map["position"].second[0]);
                    else
                        out.push_back(speak_map["position"].second[1]);
                }
                if(erry)
                {
                    if(sy > 0.0)
                        out.push_back(speak_map["position"].second[2]);
                    else
                        out.push_back(speak_map["position"].second[3]);
                }
                if(errz)
                {
                    if(sz > 0.0)
                        out.push_back(speak_map["position"].second[4]);
                    else
                        out.push_back(speak_map["position"].second[5]);
                }
                return make_pair(2,out);
            }
        }

        /*******************/
        /*   THIRD CHECK   */
        /*******************/
        //we check the error in speed
        {
            int dfx = fcx-ftx;
            int dfy = fcy-fty;
            int dfz = fcz-ftz;
            bool fxy_pos = dfx > range_freq && dfy > range_freq;
            bool fxz_pos = dfx > range_freq && dfz > range_freq;
            bool fyz_pos = dfy > range_freq && dfz > range_freq;
            bool fxy_neg = dfx < -range_freq && dfy < -range_freq;
            bool fxz_neg = dfx < -range_freq && dfz < -range_freq;
            bool fyz_neg = dfy < -range_freq && dfz < -range_freq;

            vector<string> out;
            if(fxy_pos || fxz_pos || fyz_pos)
            {
                out.push_back(speak_map["speed"].second[0]);
                return make_pair(3,out);
            }
            else if(fxy_neg || fxz_neg || fyz_neg)
            {
                out.push_back(speak_map["speed"].second[1]);
                return make_pair(3,out);
            }
        }

        //the joint passed all checks, so it is moving well
        vector<string> out;
        out.push_back("");
        return make_pair(4,out);
    }

    /********************************************************/
    void speak(const string &key, const vector<SpeechParam> &p=vector<SpeechParam>())
    {
        string value = speak_map.at(key).first;
        if(p.size() > 0)
        {
            size_t pos1 = value.find("%");
            string value_ex = value.substr(pos1+1,value.length());
            string f1 = p[0].get();
            value.replace(pos1,f1.length(),f1);
            value.erase(pos1+f1.length(),value.length());
            value += value_ex;

            if(p.size() > 1)
            {
                size_t pos2 = value.find("#");
                string f2 = p[1].get();
                string value_extra = value.substr(pos2+1,value.length());
                value.replace(pos2,f2.length(),f2);
                value += value_extra;
            }

            if(p.size() > 2)
            {
                for(size_t i=2; i<p.size(); i++)
                {
                    if(i==2)
                        value_ex.erase(value_ex.length()-2,value_ex.length()-1);
                    string value_extra;
                    value_extra = value.back();
                    value.pop_back();
                    string fi = "," + value_ex + p[i].get();
                    value.replace(value.length(),fi.length(),fi);
                    value += value_extra;
                }
            }
        }

        yWarning() << value;

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
        double dtw_thresh = rf.check("dtw_thresh",Value(0.3)).asDouble();
        double sdev_thresh = rf.check("sdev_thresh",Value(0.6)).asDouble();
        int f_static = rf.check("f_static",Value(1)).asInt();
        int range_freq = rf.check("range_freq",Value(2)).asInt();

        synthetizer = new Synthetizer(moduleName,rf.getContext(),speak_file,dtw_thresh,sdev_thresh,f_static,range_freq);
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
