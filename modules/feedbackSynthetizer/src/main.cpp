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
class BodyFeedback
{
    //levels of the hierarchy:
    //0: static joints moving or dynamic joints not moving
    //1: error in position
    //2: error in speed
    //3: no error (if no previous level is reached from any of the body parts)

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
    double dtw_thresh,mean_thresh,sx_thresh,sy_thresh,sz_thresh,psd_thresh;
    int f_static,range_freq;

public:

    /****************************************************************/
    JointFeedback()
    {
        f.resize(3,10);
    }

    /****************************************************************/
    void setName(const string &joint_name)
    {
        j = joint_name;
    }

    /****************************************************************/
    void setThresholds(const Vector &thresholds)
    {
        dtw_thresh = thresholds[0];
        mean_thresh = thresholds[1];
        sx_thresh = thresholds[2];
        sy_thresh = thresholds[3];
        sz_thresh = thresholds[4];
        f_static = (int)thresholds[5];
        range_freq = (int)thresholds[6];
        psd_thresh = thresholds[7];
    }

    /****************************************************************/
    void update(const int component, const double dtw, const double mu, const double std,
                const double skwn, const double ft, const double fc, const double maxpsdt,
                const double maxpsdc, const double std_thresh, const double psd)
    {
        f[component][0] = dtw;
        f[component][1] = mu;
        f[component][2] = std;
        f[component][3] = skwn;

        if(ft == -1 || maxpsdt > psd)
            f[component][4] = ft;
        else
            f[component][4] = 0;

        if(fc == -1 || maxpsdc > psd)
            f[component][5] = fc;
        else
            f[component][5] = 0;

        f[component][6] = maxpsdt;
        f[component][7] = maxpsdc;
        f[component][8] = std_thresh;
        f[component][9] = psd;
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

    /****************************************************************/
    Matrix getFeedbackMatrix() const { return f; }
    string getName() const { return j; }
    double getDtwThresh() const { return dtw_thresh; }
    double getMeanThresh() const { return mean_thresh; }
    double getSxThresh() const { return sx_thresh; }
    double getSyThresh() const { return sy_thresh; }
    double getSzThresh() const { return sz_thresh; }
    int getFstatic() const { return f_static; }
    int getRangeFreq() const { return range_freq; }
    double getPsdThresh() const { return psd_thresh; }
};

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
    BufferedPort<Bottle> dtwPort;
    BufferedPort<Bottle> speechPort;
    BufferedPort<Bottle> scorePort;

    double thresh_confidence;

    map<string,string> bodypart2verbal;
    map<string,pair<string,vector<string>>> speak_map;
    int idtw,imean,isdev,iskwns,ift,ifc,imaxpsdt,imaxpsdc;
    int maxlevel;
    int speak_length;
    string conj;

public:

    /********************************************************/
    Synthetizer(const string &moduleName_, const string &context, const string &speak_file, const int speak_length_,
                const double &thresh_confidence_)
    {
        moduleName = moduleName_;
        speak_length = speak_length_;
        thresh_confidence = thresh_confidence_;

        vector<string> joint;
        joint.clear();

        string armLeft,armRight,legLeft,legRight,torso,head;
        string shoulderLeft,elbowLeft,handLeft,shoulderRight,elbowRight,handRight,hipLeft,kneeLeft,ankleLeft,
                hipRight,kneeRight,ankleRight,torso_j,head_j;
        if(!load_speak(context,speak_file,armLeft,armRight,legLeft,legRight,torso,head,shoulderLeft,elbowLeft,
                       handLeft,shoulderRight,elbowRight,handRight,hipLeft,kneeLeft,ankleLeft,hipRight,kneeRight,
                       ankleRight,torso_j,head_j))
        {
            string msg="Unable to locate file";
            msg+="\""+speak_file+"\"";
            yError()<<msg;
        }

        //translate body part to verbal string
        bodypart2verbal[BodyPartTag::arm_left]=armLeft;
        bodypart2verbal[BodyPartTag::arm_right]=armRight;
        bodypart2verbal[BodyPartTag::leg_left]=legLeft;
        bodypart2verbal[BodyPartTag::leg_right]=legRight;
        bodypart2verbal[BodyPartTag::torso]=torso;
        bodypart2verbal[BodyPartTag::head]=head;

        bodypart2verbal[KeyPointTag::head]=head_j;
        bodypart2verbal[KeyPointTag::shoulder_left]=shoulderLeft;
        bodypart2verbal[KeyPointTag::elbow_left]=elbowLeft;
        bodypart2verbal[KeyPointTag::hand_left]=handLeft;
        bodypart2verbal[KeyPointTag::shoulder_right]=shoulderRight;
        bodypart2verbal[KeyPointTag::elbow_right]=elbowRight;
        bodypart2verbal[KeyPointTag::hand_right]=handRight;
        bodypart2verbal[KeyPointTag::hip_left]=hipLeft;
        bodypart2verbal[KeyPointTag::knee_left]=kneeLeft;
        bodypart2verbal[KeyPointTag::ankle_left]=ankleLeft;
        bodypart2verbal[KeyPointTag::hip_right]=hipRight;
        bodypart2verbal[KeyPointTag::knee_right]=kneeRight;
        bodypart2verbal[KeyPointTag::ankle_right]=ankleRight;
        bodypart2verbal[KeyPointTag::hip_center]=torso_j;
        bodypart2verbal[KeyPointTag::shoulder_center]=torso_j;

        idtw = 1;
        imean = 2;
        isdev = 3;
        iskwns = 4;
        ift = 6;
        ifc = 7;
        imaxpsdt = 8;
        imaxpsdc = 9;

        maxlevel = 3;
        
    }

    /********************************************************/
    ~Synthetizer()
    {
    }

    /********************************************************/
    bool open()
    {
        this->useCallback();
        BufferedPort<Bottle>::open("/" + moduleName + "/action:i");
        dtwPort.open("/" + moduleName + "/dtw:i");
        speechPort.open("/" + moduleName + "/speech:o");
        scorePort.open("/" + moduleName + "/score:o");

        return true;
    }

    /****************************************************************/
    bool load_speak(const string &context, const string &speak_file,
                    string &armLeft, string &armRight, string &legLeft, string &legRight, string &torso, string &head,
                    string &shoulderLeft, string &elbowLeft, string &handLeft, string &shoulderRight, string &elbowRight,
                    string &handRight, string &hipLeft, string &kneeLeft, string &ankleLeft, string &hipRight,
                    string &kneeRight, string &ankleRight, string &torso_j, string &head_j)
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
        if (!bGroup.check("num-sections") || !bGroup.check("conjunction"))
        {
            yError()<<"Unable to find key \"num-sections\" or \"conjunction\"";
            return false;
        }

        int num_sections = bGroup.find("num-sections").asInt();
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

        Bottle &bJoint=rf_speak.findGroup("joint");
        if (bJoint.isNull())
        {
            yError()<<"Unable to find group \"joint\"";
            return false;
        }
        shoulderLeft = bJoint.find("shoulderLeft").asString();
        elbowLeft = bJoint.find("elbowLeft").asString();
        handLeft = bJoint.find("handLeft").asString();
        shoulderRight = bJoint.find("shoulderRight").asString();
        elbowRight = bJoint.find("elbowRight").asString();
        handRight = bJoint.find("handRight").asString();
        hipLeft = bJoint.find("hipLeft").asString();
        kneeLeft = bJoint.find("kneeLeft").asString();
        ankleLeft = bJoint.find("ankleLeft").asString();
        hipRight = bJoint.find("hipRight").asString();
        kneeRight = bJoint.find("kneeRight").asString();
        ankleRight = bJoint.find("ankleRight").asString();
        torso_j = bJoint.find("torso").asString();
        head_j = bJoint.find("head").asString();

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
    void onRead(Bottle &target)
    {
        string exercise = target.get(0).asString();
        string action = target.get(1).asString();
        double confidence = target.get(2).asDouble();
        yInfo() << exercise << action << confidence;
        double score;
        if(action == exercise && confidence > thresh_confidence)
        {
            Bottle *data = dtwPort.read();
            if(Bottle *feedb = data->get(0).asList())
            {
                BodyFeedback bf(maxlevel);
                for(size_t i=0; i<feedb->size(); i++)
                {
                    //we read feedback from the input port and create the object fj
                    //which contains feedback for all the component
                    JointFeedback fj;
                    if(Bottle *joint_list = feedb->get(i).asList())
                    {
                        Vector thresholds;
                        if(Bottle *b_thresholds = joint_list->get(4).asList())
                        {
                            thresholds.resize(8);
                            thresholds[0]=b_thresholds->get(1).asDouble();
                            thresholds[1]=b_thresholds->get(2).asDouble();
                            thresholds[2]=b_thresholds->get(3).asDouble();
                            thresholds[3]=b_thresholds->get(4).asDouble();
                            thresholds[4]=b_thresholds->get(5).asDouble();
                            thresholds[5]=b_thresholds->get(6).asInt();
                            thresholds[6]=b_thresholds->get(7).asInt();
                            thresholds[7]=b_thresholds->get(8).asDouble();
                            fj.setThresholds(thresholds);
                        }

                        string joint = joint_list->get(0).asString();
                        fj.setName(joint);
                        if(Bottle *feed_x = joint_list->get(1).asList())
                        {
                            fj.update(0,feed_x->get(idtw).asDouble(),feed_x->get(imean).asDouble(),feed_x->get(isdev).asDouble(),
                                      feed_x->get(iskwns).asDouble(),feed_x->get(ift).asInt(),feed_x->get(ifc).asInt(),
                                      feed_x->get(imaxpsdt).asDouble(),feed_x->get(imaxpsdc).asDouble(),thresholds[2],thresholds[7]);
                        }
                        if(Bottle *feed_y = joint_list->get(2).asList())
                        {
                            fj.update(1,feed_y->get(idtw).asDouble(),feed_y->get(imean).asDouble(),feed_y->get(isdev).asDouble(),
                                      feed_y->get(iskwns).asDouble(),feed_y->get(ift).asInt(),feed_y->get(ifc).asInt(),
                                      feed_y->get(imaxpsdt).asDouble(),feed_y->get(imaxpsdc).asDouble(),thresholds[3],thresholds[7]);
                        }
                        if(Bottle *feed_z = joint_list->get(3).asList())
                        {
                            fj.update(2,feed_z->get(idtw).asDouble(),feed_z->get(imean).asDouble(),feed_z->get(isdev).asDouble(),
                                      feed_z->get(iskwns).asDouble(),feed_z->get(ift).asInt(),feed_z->get(ifc).asInt(),
                                      feed_z->get(imaxpsdt).asDouble(),feed_z->get(imaxpsdc).asDouble(),thresholds[4],thresholds[7]);
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
                vector<pair<string,vector<SpeechParam>>> speak_buffer;
                switch (fin_level)
                {
                case -1: //moved from initial position
                    score = 0.0;
                    params.clear();
                    speak_buffer.clear();
                    speak_buffer.push_back(make_pair("position-center",params));
                    break;
                case 0: //static joints moving or dynamic joints not moving
                    score = 0.0;
                    speak_buffer.clear();
                    for(size_t i=0; i<bp.size(); i++)
                    {
                        params.clear();
                        params.push_back(bodypart2verbal[bp[i]]);
                        if(finalf[i][0] == "static")
                        {
                            speak_buffer.push_back(make_pair("static",params));
                        }
                        else
                        {
                            speak_buffer.push_back(make_pair("dynamic",params));
                        }
                    }
                    break;

                case 1: //error in speed
                    score = 0.5;
                    speak_buffer.clear();
                    for(size_t i=0; i<bp.size(); i++)
                    {
                        params.clear();
                        params.push_back(bodypart2verbal[bp[i]]);
                        for(size_t j=0; j<finalf[i].size(); j++)
                        {
                            params.push_back(finalf[i][j]);
                        }
                        speak_buffer.push_back(make_pair("speed",params));
                    }
                    break;

                case 2: //error in position
                    score = 0.5;
                    speak_buffer.clear();
                    for(size_t i=0; i<bp.size(); i++)
                    {
                        params.clear();
                        params.push_back(bodypart2verbal[bp[i]]);
                        for(size_t j=0; j<finalf[i].size(); j++)
                        {
                            params.push_back(finalf[i][j]);
                        }

                        speak_buffer.push_back(make_pair("position",params));
                    }
                    break;

                case 3: //no error
                    score = 1.0;
                    params.clear();
                    speak_buffer.clear();
                    speak_buffer.push_back(make_pair("perfect",params));
                    break;
                }

                speak(speak_buffer);
                cout << endl;
            }
        }
        else
        {
            vector<pair<string,vector<SpeechParam>>> speak_buffer;
            vector<SpeechParam> params;
            speak_buffer.clear();
            params.clear();
            speak_buffer.push_back(make_pair("wrong",params));
            speak(speak_buffer);
            score = 0.0;
        }

        Bottle &outscore = scorePort.prepare();
        outscore.clear();
        outscore.addDouble(score);
        scorePort.write();
    }

    /********************************************************/
    pair<int,vector<string>> assess(JointFeedback f)
    {
        Matrix feedb = f.getFeedbackMatrix();
        double dx = feedb[0][0];
        double mx = feedb[0][1];
        double kx = feedb[0][2];
        double sx = feedb[0][3];
        int ftx = (int)feedb[0][4];
        int fcx = (int)feedb[0][5];

        double dy = feedb[1][0];
        double my = feedb[1][1];
        double ky = feedb[1][2];
        double sy = feedb[1][3];
        int fty = (int)feedb[1][4];
        int fcy = (int)feedb[1][5];

        double dz = feedb[2][0];
        double mz = feedb[2][1];
        double kz = feedb[2][2];
        double sz = feedb[2][3];
        int ftz = (int)feedb[2][4];
        int fcz = (int)feedb[2][5];

        /*******************/
        /*   ZERO CHECK    */
        /*******************/
        /*{
            string joint = f.getName();
            bool errcenterx = fabs(mx) > f.getMeanThresh();
            bool errcentery = fabs(my) > f.getMeanThresh();
            bool errcenterz = fabs(mz) > f.getMeanThresh();
            if(joint == KeyPointTag::shoulder_center && (errcenterx || errcentery || errcenterz))
            {
                vector<string> out;
                out.push_back("");
                return make_pair(-1,out);
            }
        } */       
         
        /*******************/
        /*   FIRST CHECK   */
        /*******************/
        //static joints in the template must not move
        //dynamic joints in the template must move
        bool joint_templ_stale = ftx < 0 && fty < 0 && ftz < 0;
        bool joint_skel_stale = fcx < 0 && fcy < 0 && fcz < 0;
        /* if(!joint_templ_stale && !joint_skel_stale)
        {
            {
                bool ftxy = ftx <= f.getFstatic() && fty <= f.getFstatic();
                bool ftxz = ftx <= f.getFstatic() && ftz <= f.getFstatic();
                bool ftyz = fty <= f.getFstatic() && ftz <= f.getFstatic();

                bool fcxy = fcx > f.getFstatic() && fcy > f.getFstatic();
                bool fcxz = fcx > f.getFstatic() && fcz > f.getFstatic();
                bool fcyz = fcy > f.getFstatic() && fcz > f.getFstatic();

                //we check if the difference is significant
                bool dfx = fabs(ftx-fcx) > f.getRangeFreq();
                bool dfy = fabs(fty-fcy) > f.getRangeFreq();
                bool dfz = fabs(ftz-fcz) > f.getRangeFreq();

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
        }

        if(!joint_templ_stale && !joint_skel_stale)
        {
            bool ftxy = ftx > f.getFstatic() && fty > f.getFstatic();
            bool ftxz = ftx > f.getFstatic() && ftz > f.getFstatic();
            bool ftyz = fty > f.getFstatic() && ftz > f.getFstatic();

            bool fcxy = fcx <= f.getFstatic() && fcy <= f.getFstatic();
            bool fcxz = fcx <= f.getFstatic() && fcz <= f.getFstatic();
            bool fcyz = fcy <= f.getFstatic() && fcz <= f.getFstatic();

            //we check if the difference is significant
            bool dfx = fabs(ftx-fcx) >=  f.getRangeFreq();
            bool dfy = fabs(fty-fcy) >=  f.getRangeFreq();
            bool dfz = fabs(ftz-fcz) >=  f.getRangeFreq();

            if(ftxy && fcxy)
            {
                if(dfx && dfy)
                {
                    vector<string> out;
                    out.push_back("dynamic");
                    return make_pair(0,out);
                }
            }
            if(ftxz && fcxz)
            {
                if(dfx && dfz)
                {
                    vector<string> out;
                    out.push_back("dynamic");
                    return make_pair(0,out);
                }
            }
            if(ftyz && fcyz)
            {
                if(dfy && dfz)
                {
                    vector<string> out;
                    out.push_back("dynamic");
                    return make_pair(0,out);
                }
            }
        } */

        /********************/
        /*   SECOND CHECK   */
        /********************/
        //we check the error in speed
        if(!joint_templ_stale && !joint_skel_stale)
        {
            int dfx = fcx-ftx;
            int dfy = fcy-fty;
            int dfz = fcz-ftz;
            bool fxy_pos = dfx >  f.getRangeFreq() && dfy >  f.getRangeFreq();
            bool fxz_pos = dfx >  f.getRangeFreq() && dfz >  f.getRangeFreq();
            bool fyz_pos = dfy >  f.getRangeFreq() && dfz >  f.getRangeFreq();
            bool fxy_neg = dfx < - f.getRangeFreq() && dfy < - f.getRangeFreq();
            bool fxz_neg = dfx < - f.getRangeFreq() && dfz < - f.getRangeFreq();
            bool fyz_neg = dfy < - f.getRangeFreq() && dfz < - f.getRangeFreq();

            vector<string> out;
            if(fxy_pos || fxz_pos || fyz_pos)
            {
                out.push_back(speak_map["speed"].second[0]);
                return make_pair(1,out);
            }
            else if(fxy_neg || fxz_neg || fyz_neg)
            {
                out.push_back(speak_map["speed"].second[1]);
                return make_pair(1,out);
            }
        }

        /*******************/
        /*   THIRD CHECK   */
        /*******************/
        //we check the error in position
        if(!joint_templ_stale && !joint_skel_stale)
        {
            bool dtwx = dx < f.getDtwThresh();
            bool dtwy = dy < f.getDtwThresh();
            bool dtwz = dz < f.getDtwThresh();
            
            bool errx = kx > f.getSxThresh();
            bool erry = ky > f.getSyThresh();
            bool errz = kz > f.getSzThresh();                 
                                   
            if( errx || erry || errz )
            {
                vector<string> out;
                if(errx)
                {
                    if(sx > 0.0)
                        out.push_back(speak_map["position"].second[0]);
                    else
                        out.push_back(speak_map["position"].second[1]);
                    return make_pair(2,out);
                }
                if(erry)
                {
                    if(sy > 0.0)
                        out.push_back(speak_map["position"].second[2]);
                    else
                        out.push_back(speak_map["position"].second[3]);
                    return make_pair(2,out);
                }
                if(errz)
                {
                    if(sz > 0.0)
                        out.push_back(speak_map["position"].second[4]);
                    else
                        out.push_back(speak_map["position"].second[5]);
                    return make_pair(2,out);
                }
            }
        }

        //the joint passed all checks, so it is moving well
        vector<string> out;
        out.push_back("");
        return make_pair(3,out);
    }

    /********************************************************/
    void speak(const vector<pair<string,vector<SpeechParam>>> &buffer)
    {
        for(size_t i=0; i<buffer.size(); i++)
        {
            if(i>=speak_length)
                break;
            string key = buffer[i].first;
            string value = speak_map.at(key).first;
            vector<SpeechParam> p = buffer[i].second;
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
                        string fi = conj + value_ex + p[i].get();
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
        dtwPort.close();
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
        double thresh_confidence = rf.check("thresh-conf",Value(0.7)).asDouble();

        synthetizer = new Synthetizer(moduleName,rf.getContext(),speak_file,speak_length,thresh_confidence);
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
