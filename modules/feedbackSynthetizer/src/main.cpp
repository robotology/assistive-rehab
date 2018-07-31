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
class FeedbackParam
{
    string j;
    Matrix f;

public:

    /****************************************************************/
    FeedbackParam()
    {
        f.resize(3,4);
    }

    /****************************************************************/
    void setName(const string &joint_name)
    {
        j = joint_name;
    }

    /****************************************************************/
    void update(const int component, const double k, const double s, const double ft, const double fc)
    {
        f[component][0] = k;
        f[component][1] = s;
        f[component][2] = ft;
        f[component][3] = fc;
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

    Matrix getFeedback() const { return f; }
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
class Feedback : public BufferedPort<Bottle>
{
    string moduleName;
    BufferedPort<Bottle> speechPort;

    double var_thresh,skwns_thresh;
    int range_static,range_freq;
    map<string,string> joint2bodypart;
    map<string,string> bodypart2verbal;
    map<string,pair<string,vector<string>>> speak_map;
    int idtw,ivar,iskwns,ift,ifc;
    int maxlevel;

public:

    /********************************************************/
    Feedback(const string &moduleName_, const string &context, const string &speak_file,
             const double &var_thresh_, const double &skwns_thresh_, const int range_static_,
             const int range_freq_)
    {
        moduleName = moduleName_;
        var_thresh = var_thresh_;
        skwns_thresh = skwns_thresh_;
        range_static = range_static_;
        range_freq = range_freq_;

        vector<string> body;
        body.clear();
        if(!load_speak(context,speak_file,body))
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
        bodypart2verbal[BodyPartTag::arm_left]=body[0];
        bodypart2verbal[BodyPartTag::arm_right]=body[1];
        bodypart2verbal[BodyPartTag::leg_left]=body[2];
        bodypart2verbal[BodyPartTag::leg_right]=body[3];
        bodypart2verbal[BodyPartTag::torso]=body[4];
        bodypart2verbal[BodyPartTag::head]=body[5];

        idtw = 1;
        ivar = 2;
        iskwns = 3;
        ift = 5;
        ifc = 6;

        maxlevel = 4;
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
    bool load_speak(const string &context, const string &speak_file, vector<string> &body)
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
        if (!bGroup.check("num-sections") || !bGroup.check("body"))
        {
            yError()<<"Unable to find key \"num-sections\" and/or \"body\"";
            return false;
        }

        int num_sections = bGroup.find("num-sections").asInt();
        Bottle *bBody = bGroup.find("body").asList();
        for(int i=0; i<bBody->size(); i++)
        {
            body.push_back(bBody->get(i).asString());
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

            map<string,pair<int,string>> xyz2hierarchy;
            xyz2hierarchy.clear(); 
            for(size_t i=0; i<feedb->size(); i++)
            {
                //we read feedback from the input port and create the object fj
                //which contains feedback for all the component
                FeedbackParam fj;
                if(Bottle *joint_list = feedb->get(i).asList())
                {
                    string joint = joint_list->get(0).asString();
                    fj.setName(joint);
                    if(Bottle *feed_x = joint_list->get(1).asList())
                    {
                        fj.update(0,feed_x->get(ivar).asDouble(),feed_x->get(iskwns).asDouble(),
                                  feed_x->get(ift).asInt(),feed_x->get(ifc).asInt());
                    }
                    if(Bottle *feed_y = joint_list->get(2).asList())
                    {
                        fj.update(1,feed_y->get(ivar).asDouble(),feed_y->get(iskwns).asDouble(),
                                  feed_y->get(ift).asInt(),feed_y->get(ifc).asInt());
                    }
                    if(Bottle *feed_z = joint_list->get(3).asList())
                    {
                        fj.update(2,feed_z->get(ivar).asDouble(),feed_z->get(iskwns).asDouble(),
                                  feed_z->get(ift).asInt(),feed_z->get(ifc).asInt());
                    }

                    //FIRST LEVEL OF INTEGRATION
                    //from xyz to a level of priority for the single joint
                    pair<int,string> feed_jnt = assess(fj);
                    if(feed_jnt.first != 3)
                    {
                        yInfo() << fj.getName() << feed_jnt.first << feed_jnt.second;
                        fj.print();
                    }
                    xyz2hierarchy[fj.getName()] = feed_jnt;
                }
            }
            cout << endl;

            //SECOND LEVEL OF INTEGRATION: from single joint to bodypart
            //this structure associates to each body part a pair that contains:
            //a vector: the level of priority and the number of joints with the minimum level of priority
            //a string: the verbal feedback for that level of priority
            map<string,pair<vector<int>,string>> bodypart2feedback;
            bodypart2feedback.clear();

            //initialize structure
            vector<int> init;
            init.clear();
            init.push_back(maxlevel);
            init.push_back(0);
            bodypart2feedback[BodyPartTag::arm_left] = make_pair(init,"");
            bodypart2feedback[BodyPartTag::arm_right] = make_pair(init,"");
            bodypart2feedback[BodyPartTag::leg_left] = make_pair(init,"");
            bodypart2feedback[BodyPartTag::leg_right] = make_pair(init,"");
            bodypart2feedback[BodyPartTag::torso] = make_pair(init,"");
            bodypart2feedback[BodyPartTag::head] = make_pair(init,"");

            //we find the minimum level associated to each body part
            for(auto &it: xyz2hierarchy)
            {
                string bp = joint2bodypart[it.first];
                string ftot = it.second.second;
                int h = it.second.first;
                if(h <= bodypart2feedback[bp].first[0])
                {
                    vector<int> t;
                    t.clear();
                    t.push_back(h);
                    t.push_back(0);
                    bodypart2feedback[bp] = make_pair(t,ftot);
                }
            }

            //and we count how many joints of that body part are at that level
            int fin_level = maxlevel;
            for(auto &it: xyz2hierarchy)
            {
                string jnt = it.first;
                int jnt_level = it.second.first;
                string bp = joint2bodypart[jnt];
                int min_level = bodypart2feedback[bp].first[0];
                if(jnt_level == min_level)
                {
                     int count = bodypart2feedback[bp].first[1]+1;
                     bodypart2feedback[bp].first[1] = count;               
                }
                if(min_level < fin_level)
                    fin_level = min_level;
            }              

//            int fin_level = maxlevel;
//            for(auto &it: bodypart2feedback)
//            {
//                int level = it.second.first[0];
//                if(level < fin_level)
//                {
//                    fin_level = level;
//                }
//            }

            for(auto &it: bodypart2feedback)
            {
                if(it.second.first[0] != 3)
                    yInfo() << it.first << it.second.first[0] << it.second.first[1] << it.second.second;
            }

            //we finally have the list of body parts that have to be adjusted and the related feedback
            vector<string> bp,finalf;
            bp.clear();
            finalf.clear();
            for(auto &it: bodypart2feedback)
            {
                int level = it.second.first[0];
                //int njnts = it.second.first[1];
                if(level == fin_level)
                {
                    //if(level == 0)
                    //{
                        //if static joints are moving, at least two of them have two move to provide the feedback "static"
                        //if(it.second.second == "static")
                        //{
                        //    if(njnts >= 2)
                        //    {
                        //        bp.push_back(bodypart2verbal[it.first]);
                        //        finalf.push_back(it.second.second);
                        //    }
                        //    else
                        //        fin_level = maxlevel;
                        //}
                        //else
                        //{
                        //    bp.push_back(bodypart2verbal[it.first]);
                        //    finalf.push_back(it.second.second);
                        //}
                    //}
                    //otherwise it's likely that the exercise is being performed wrongly ==> feedback = "wrong"
                    //else
                    //{
                        bp.push_back(bodypart2verbal[it.first]);
                        finalf.push_back(it.second.second);
                    //}
                }
            }

            //speak
            vector<SpeechParam> params;
            switch (fin_level)
            {
            case 0: //static joints moving or dynamic joints not moving
                for(size_t i=0; i<bp.size(); i++)
                {
                    params.clear();
                    params.push_back(bp[i]);
                    if(finalf[i] == "static")
                        speak("static",params);
                    else
                        speak("dynamic",params);
                }
                break;

            case 1: //error in position
                for(size_t i=0; i<bp.size(); i++)
                {
                    params.clear();
                    params.push_back(bp[i]);
                    params.push_back(finalf[i]);
                    speak("position",params);
                }
                break;

            case 2: //error in speed
                for(size_t i=0; i<bp.size(); i++)
                {
                    params.clear();
                    params.push_back(bp[i]);
                    params.push_back(finalf[i]);
                    speak("speed",params);
                }
                break;

            case 3: //no error
                speak("perfect");
                break;

            case 4: //wrong movement
                speak("wrong");
                break;
            }
            cout << endl;

            //we use this structure for providing a feedback
            //            assess(bodypart2feed);
        }

    }

    /********************************************************/
//    void print(const map<string,vector<double>> &bodypart2feed)
//    {
//        for(auto &it : bodypart2feed)
//        {
//            string bodypart = it.first;
//            double dx = it.second[ibpd_x];
//            double var_x = it.second[ibpvar_x];
//            double skwns_x = it.second[ibpskwns_x];
//            int ftx = it.second[ift_x];
//            int fcx = it.second[ifc_x];
//            double dy = it.second[ibpd_y];
//            double var_y = it.second[ibpvar_y];
//            double skwns_y = it.second[ibpskwns_y];
//            int fty = it.second[ift_y];
//            int fcy = it.second[ifc_y];
//            double dz = it.second[ibpd_z];
//            double var_z = it.second[ibpvar_z];
//            double skwns_z = it.second[ibpskwns_z];
//            int ftz = it.second[ift_z];
//            int fcz = it.second[ifc_z];

//            cout << bodypart
//                 << " ( x "   << dx << " " << var_x << " " << skwns_x << " )"
//                 << " ( fx " << ftx   << " " << fcx << " )"
//                 << " ( y " << dy << " " << var_y << " " << skwns_y << " )"
//                 << " ( fy " << fty  << " " << fcy << " )"
//                 << " ( z " << dz << " " << var_z << " " << skwns_z << " )"
//                 << " ( fz " << ftz   << " " << fcz << " )"
//                 << endl;
//        }
//    }

    /********************************************************/
    pair<int,string> assess(FeedbackParam f)
    {
        Matrix feedb = f.getFeedback();
        double kx = feedb[0][0];
        double sx = feedb[0][1];
        int ftx = feedb[0][2];
        int fcx = feedb[0][3];

        double ky = feedb[1][0];
        double sy = feedb[1][1];
        int fty = feedb[1][2];
        int fcy = feedb[1][3];

        double kz = feedb[2][0];
        double sz = feedb[2][1];
        int ftz = feedb[2][2];
        int fcz = feedb[2][3];

        /*******************/
        /*   FIRST CHECK   */
        /*******************/
        //static joints in the template must not move
        //dynamic joints in the template must move
        {
            bool ftxy = ftx < range_static && fty < range_static;
            bool ftxz = ftx < range_static && ftz < range_static;
            bool ftyz = fty < range_static && ftz < range_static;

            bool fcxy = fcx >= range_static && fcy >= range_static;
            bool fcxz = fcx >= range_static && fcz >= range_static;
            bool fcyz = fcy >= range_static && fcz >= range_static;

            if( (ftxy && fcxy) || (ftxz && fcxz) || (ftyz && fcyz) )
            {
                return make_pair(0,"static");
            }
        }

        {
            bool ftxy = ftx >= range_static && fty >= range_static;
            bool ftxz = ftx >= range_static && ftz >= range_static;
            bool ftyz = fty >= range_static && ftz >= range_static;

            bool fcxy = fcx < range_static && fcy < range_static;
            bool fcxz = fcx < range_static && fcz < range_static;
            bool fcyz = fcy < range_static && fcz < range_static;

            if( (ftxy && fcxy) || (ftxz && fcxz) || (ftyz && fcyz) )
            {
                return make_pair(0,"dynamic");
            }
        }

        /********************/
        /*   SECOND CHECK   */
        /********************/
        //we check the error in position
        {
            bool errx = kx > var_thresh;
            bool erry = ky > var_thresh;
            bool errz = kz > var_thresh;
            if(errx || erry || errz)
            {
                if(errx)
                {
                    if(sx > 0.0)
                        return make_pair(1,speak_map["position"].second[0]);
                    else
                        return make_pair(1,speak_map["position"].second[1]);
                }
                if(erry)
                {
                    if(sy > 0.0)
                        return make_pair(1,speak_map["position"].second[3]);
                    else
                        return make_pair(1,speak_map["position"].second[4]);
                }
                if(errz)
                {
                    if(sz > 0.0)
                        return make_pair(1,speak_map["position"].second[6]);
                    else
                        return make_pair(1,speak_map["position"].second[7]);
                }
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
            if(fxy_pos || fxz_pos || fyz_pos)
            {
                return make_pair(2,speak_map["speed"].second[0]);
            }
            else if(fxy_neg || fxz_neg || fyz_neg)
            {
                return make_pair(2,speak_map["speed"].second[1]);
            }
        }

        //the joint passed all checks, so it is moving well
        return make_pair(3,"");
    }

    /********************************************************/
/*    bool assess(const map<string,vector<double>> &bodypart2feed)
    {
        vector<string> err_static,err_dynamic;
        vector<pair<string,string>> err_pos,err_speed;
        err_static.clear();
        err_dynamic.clear();
        err_pos.clear();
        err_speed.clear();
        for(auto &it : bodypart2feed)
        {
            string bodypart = it.first;

            int ftx = it.second[ift_x];
            int fcx = it.second[ifc_x];
            int fty = it.second[ift_y];
            int fcy = it.second[ifc_y];
            int ftz = it.second[ift_z];
            int fcz = it.second[ifc_z];

            //error static joints that move
            if(ftx <= range_static && fty <= range_static && ftz <= range_static)
            {
                bool xy = fcx > range_static && fcy > range_static;
                bool xz = fcx > range_static && fcz > range_static;
                bool yz = fcy > range_static && fcz > range_static;
                if(xy || xz || yz)
                {
                    err_static.push_back(bodypart2verbal[bodypart]);
                }
            }
            else
            {
                //error dynamic joints that do not move
                bool xy = ftx > range_static && fty > range_static;
                bool xz = ftx > range_static && ftz > range_static;
                bool yz = fty > range_static && ftz > range_static;
                if(xy && (fcx < range_static && fcy < range_static))
                {
                    err_dynamic.push_back(bodypart2verbal[bodypart]);
                }
                else if(xz && (fcx < range_static && fcz < range_static))
                {
                    err_dynamic.push_back(bodypart2verbal[bodypart]);
                }
                else if(yz && (fcy < range_static && fcz < range_static))
                {
                    err_dynamic.push_back(bodypart2verbal[bodypart]);
                }

                //error position
                double kurt_x = it.second[ibpvar_x];
                double skwns_x = it.second[ibpskwns_x];
                if(kurt_x > var_thresh)
                {
                    if(skwns_x > 0.0)
                    {
                        err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[0]));
                    }
                    else
                    {
                        err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[1]));
                    }
                }

//                if(var_x < var_thresh && skwns_x > skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[0]));
//                }
//                else if(var_x < var_thresh && skwns_x < -skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[1]));
//                }
//                else if(var_x > var_thresh && fabs(skwns_x) < skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[2]));
//                }

                double kurt_y = it.second[ibpvar_y];
                double skwns_y = it.second[ibpskwns_y];
                if(kurt_y > var_thresh)
                {
                    if(skwns_y > 0.0)
                    {
                        err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[3]));
                    }
                    else
                    {
                        err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[4]));
                    }
                }

//                if(var_y < var_thresh && skwns_y > skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[3]));
//                }
//                else if(var_y < var_thresh && skwns_y < -skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[4]));
//                }
//                else if(var_y > var_thresh && fabs(skwns_y) < skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[5]));
//                }

                double kurt_z = it.second[ibpvar_z];
                double skwns_z = it.second[ibpskwns_z];
                if(kurt_z > var_thresh)
                {
                    if(skwns_z > 0.0)
                    {
                        err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[6]));
                    }
                    else
                    {
                        err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[7]));
                    }
                }

//                if(var_z < var_thresh && skwns_z > skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[6]));
//                }
//                else if(var_z < var_thresh && skwns_z < -skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[7]));
//                }
//                else if(var_z > var_thresh && fabs(skwns_z) < skwns_thresh)
//                {
//                    err_pos.push_back(make_pair(bodypart2verbal[bodypart],speak_map["position"].second[8]));
//                }

                //error speed
                int dfx = fcx-ftx;
                int dfy = fcy-fty;
                int dfz = fcz-ftz;
                bool fxy_pos = dfx > range_freq && dfy > range_freq;
                bool fxz_pos = dfx > range_freq && dfz > range_freq;
                bool fyz_pos = dfy > range_freq && dfz > range_freq;
                bool fxy_neg = dfx < -range_freq && dfy < -range_freq;
                bool fxz_neg = dfx < -range_freq && dfz < -range_freq;
                bool fyz_neg = dfy < -range_freq && dfz < -range_freq;
                if(fxy_pos || fxz_pos || fyz_pos)
                {
                    err_speed.push_back(make_pair(bodypart2verbal[bodypart],speak_map["speed"].second[0]));
                }
                else if(fxy_neg || fxz_neg || fyz_neg)
                {
                    err_speed.push_back(make_pair(bodypart2verbal[bodypart],speak_map["speed"].second[1]));
                }
            }
        }

        //first check to be passed:
        //all static body parts are not moving
        //all dynamic body parts are moving
        if(err_static.size() > 0 || err_dynamic.size() > 0)
        {
            if(err_static.size() > 0)
            {
                vector<SpeechParam> params;
                for(int i=0; i<err_static.size(); i++)
                {
                    params.clear();
                    params.push_back(err_static[i]);
                    speak("static",params);
                }
            }

            if(err_dynamic.size() > 0)
            {
                vector<SpeechParam> params;
                for(int i=0; i<err_dynamic.size(); i++)
                {
                    params.clear();
                    params.push_back(err_dynamic[i]);
                    speak("dynamic",params);
                }
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

    } */

    /********************************************************/
    void speak(const string &key, const vector<SpeechParam> &p=vector<SpeechParam>())
    {
        string value = speak_map.at(key).first;
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
    Feedback *feedback;
    double period;

public:

    /********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string moduleName = rf.check("name",Value("feedbackSynthetizer")).asString();
        setName(moduleName.c_str());

        string speak_file=rf.check("speak-file",Value("speak-it")).asString();
        period = rf.check("period",Value(0.1)).asDouble();
        double var_thresh = rf.check("var_thresh",Value(5.0)).asDouble();
        double skwns_thresh = rf.check("skwns_thresh",Value(1.5)).asDouble();
        int range_static = rf.check("range_static",Value(2)).asInt();
        int range_freq = rf.check("range_freq",Value(2)).asInt();

        feedback = new Feedback(moduleName,rf.getContext(),speak_file,var_thresh,skwns_thresh,range_static,range_freq);
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
