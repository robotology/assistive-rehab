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
#include <memory>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include "AssistiveRehab/skeleton.h"
#include "src/attentionManager_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;


/****************************************************************/
class Attention : public RFModule, public attentionManager_IDL
{
    enum class State { unconnected, connection_trigger, idle, seek, follow } state;
    bool auto_mode,virtual_mode;

    const int ack=Vocab::encode("ack");
    const double T=3.0;
    double period;
    double gaze_follow_T;
    double gaze_seek_T;
    double inactivity_thres;
    double still_t0;
    double lost_t0;
    bool first_follow_look;
    bool first_seek_look;

    Matrix gaze_frame,gaze_frame_init;
    bool first_gaze_frame;
    Vector is_following_x;
    vector<shared_ptr<Skeleton>> skeletons;
    vector<double> activity;
    string tag;
    string keypoint;

    Mutex mutex;
    BufferedPort<Bottle> opcPort;
    RpcClient gazeCmdPort;
    BufferedPort<Property> gazeStatePort;
    RpcServer cmdPort;

    class Reporter : public PortReport {
        Attention &attention;
        bool first_run;
        void report(const PortInfo &info) override {
            if (first_run && info.created && !info.incoming) {
                attention.state=State::connection_trigger;
                first_run=false;
            }
        }
    public:
        Reporter(Attention &attention_) : attention(attention_), first_run(true) { }
    } reporter;

    /****************************************************************/
    bool look(const string &tag, const string &keypoint) override
    {
        LockGuard lg(mutex);
        if (state<State::idle)
        {
            return false;
        }
        this->keypoint=keypoint;
        auto_mode=false;
        if ((state==State::follow) && (this->tag==tag))
        {
            return true;
        }
        else
        {
            this->tag=tag;
            return switch_to(State::seek);
        }
    }

    /****************************************************************/
    bool stop() override
    {
        LockGuard lg(mutex);
        if (state<State::idle)
        {
            return false;
        }
        bool ret=true;
        ret&=switch_to(state=State::idle);
        ret&=look("angular",zeros(2));
        first_gaze_frame=true;
        return ret;
    }

    /****************************************************************/
    bool is_running() override
    {
        LockGuard lg(mutex);
        if (state<State::idle)
        {
            return false;
        }
        return (state!=State::idle);
    }

    /****************************************************************/
    FollowedSkeletonInfo is_following() override
    {
        LockGuard lg(mutex);
        FollowedSkeletonInfo info;
        if (state==State::follow)
        {
            info.tag=tag;
            info.x=is_following_x[0];
            info.y=is_following_x[1];
            info.z=is_following_x[2];
        }
        else
        {
            info.tag=string("");
            info.x=info.y=info.z=0.0;
        }
        return info;
    }

    /****************************************************************/
    vector<string> is_any_raised_hand() override
    {
        LockGuard lg(mutex);
        vector<string> ret;
        if (state>=State::idle)
        {
            for (auto &s:skeletons)
            {
                if (is_with_raised_hand(s))
                {
                    ret.push_back(s->getTag());
                }
            }
        }
        return ret;
    }

    /****************************************************************/
    bool is_with_raised_hand(const string &tag) override
    {
        LockGuard lg(mutex);
        if (state<State::idle)
        {
            return false;
        }
        for (auto &s:skeletons)
        {
            if (s->getTag()==tag)
            {
                return is_with_raised_hand(s);
            }
        }
        return false;
    }

    /****************************************************************/
    bool set_auto() override
    {
        LockGuard lg(mutex);
        if (state<State::idle)
        {
            return false;
        }
        auto_mode=true;
        return switch_to(State::seek);
    }

    /****************************************************************/
    bool set_virtual() override
    {
        LockGuard lg(mutex);
        virtual_mode=true;
        return true;
    }

    /****************************************************************/
    shared_ptr<Skeleton> find_skeleton_tag(const string &tag) const
    {
        for (auto &s:skeletons)
        {
            if (s->getTag()==tag)
            {
                return s;
            }
        }
        return nullptr;
    }

    /****************************************************************/
    bool is_with_raised_hand(const shared_ptr<Skeleton> &s) const
    {
        if ((*s)[KeyPointTag::elbow_left]->isUpdated() &&
            (*s)[KeyPointTag::hand_left]->isUpdated())
        {
            if (((*s)[KeyPointTag::elbow_left]->getPoint()[1]>
                 (*s)[KeyPointTag::hand_left]->getPoint()[1]))
                return true;
        }
        if ((*s)[KeyPointTag::elbow_right]->isUpdated() &&
            (*s)[KeyPointTag::hand_right]->isUpdated())
        {
            if (((*s)[KeyPointTag::elbow_right]->getPoint()[1]>
                 (*s)[KeyPointTag::hand_right]->getPoint()[1]))
                return true;
        }
        return false;
    }

    /****************************************************************/
    shared_ptr<Skeleton> find_skeleton_raised_hand() const
    {
        for (auto &s:skeletons)
        {
            if (is_with_raised_hand(s))
            {
                return s;
            }
        }
        return nullptr;
    }

    /****************************************************************/
    bool is_inactive(const shared_ptr<Skeleton> &s)
    {
        double a=0.0;
        unsigned int n=0;
        for (unsigned int i=0; i<s->getNumKeyPoints(); i++)
        {
            if ((*s)[i]->isUpdated())
            {
                a+=norm((*s)[i]->getPoint());
                n++;
            }
        }
        activity.push_back(a/n);

        if (activity.size()>=(2.0*T)/getPeriod())
        {
            double mean=0.0;
            double stdev=0.0;
            for (auto &p:activity)
            {
                mean+=p;
                stdev+=p*p;
            }
            mean/=activity.size();
            stdev=sqrt(stdev/activity.size()-mean*mean);
            activity.clear();
            yInfo()<<"Inactivity check of"<<s->getTag()
                   <<":"<<stdev<<"<"<<inactivity_thres<<"?";
            return (stdev<inactivity_thres);
        }
        else
        {
            return false;
        }
    }

    /****************************************************************/
    bool switch_to(const State &next_state)
    {
        state=next_state;
        if (state==State::seek)
        {
            first_seek_look=true;
        }
        return set_gaze_T(state==State::follow?gaze_follow_T:gaze_seek_T);
    }

    /****************************************************************/
    bool seek()
    {
        if (auto_mode)
        {
            random_shuffle(begin(skeletons),end(skeletons));
            if (auto s=find_skeleton_raised_hand())
            {
                tag=s->getTag();
            }
            else
            {
                for (auto &s:skeletons)
                {
                    tag=s->getTag();
                    if (tag[0]!='#')
                    {
                        break;
                    }
                }
            }
            keypoint=KeyPointTag::head;
            if (!skeletons.empty())
            {
                yInfo()<<"[Auto]: follow"<<tag;
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return (find_skeleton_tag(tag)!=nullptr);
        }
    }

    /****************************************************************/
    bool set_gaze_T(const double T) const
    {
        Bottle cmd,rep;
        cmd.addVocab(Vocab::encode("set"));
        cmd.addString("T");
        cmd.addDouble(T);
        if (gazeCmdPort.write(cmd,rep))
        {
            return (rep.get(0).asVocab()==ack);
        }
        return false;
    }

    /****************************************************************/
    bool look(const string &type, const Vector &v) const
    {
        Bottle loc;
        loc.addList().read(v);

        Property options;
        options.put("control-frame","depth_rgb");
        options.put("target-type",type);
        if (type=="image")
        {
            options.put("image","depth_rgb");
        }
        options.put("target-location",loc.get(0));

        Bottle cmd,rep;
        cmd.addVocab(Vocab::encode("look"));
        cmd.addList().read(options);
        if (gazeCmdPort.write(cmd,rep))
        {
            return (rep.get(0).asVocab()==ack);
        }
        return false;
    }

    /****************************************************************/
    bool wait_motion_done()
    {
        Bottle cmd;
        cmd.addVocab(Vocab::encode("get"));
        cmd.addVocab(Vocab::encode("done"));

        const double t0=Time::now();
        while (Time::now()-t0<T)
        {
            Time::delay(getPeriod());

            Bottle rep;
            if (gazeCmdPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==ack)
                {
                    if (rep.get(1).asInt()>0)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /****************************************************************/
    bool attach(RpcServer &source) override
    {
        return yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        auto_mode=rf.check("auto-start");
        period=rf.check("period",Value(0.1)).asDouble();
        gaze_follow_T=rf.check("gaze-follow-T",Value(1.5)).asDouble();
        gaze_seek_T=rf.check("gaze-seek-T",Value(2.0)).asDouble();
        inactivity_thres=rf.check("inactivity-thres",Value(0.05)).asDouble();
        virtual_mode=rf.check("virtual-mode",Value(false)).asBool();

        opcPort.open("/attentionManager/opc:i");
        gazeCmdPort.open("/attentionManager/gaze/cmd:rpc");
        gazeStatePort.open("/attentionManager/gaze/state:i");
        cmdPort.open("/attentionManager/cmd:rpc");

        gazeCmdPort.setReporter(reporter);
        attach(cmdPort);

        state=State::unconnected;
        is_following_x.resize(3,0.0);
        still_t0=lost_t0=Time::now();
        first_follow_look=false;
        first_seek_look=false;
        first_gaze_frame=true;

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
        LockGuard lg(mutex);
        if (Property *p=gazeStatePort.read(false))
        {
            Vector pose;
            p->find("depth_rgb").asList()->write(pose);
            gaze_frame=axis2dcm(pose.subVector(3,6));
            gaze_frame.setSubcol(pose.subVector(0,2),0,3);
        }

        if(first_gaze_frame && virtual_mode)
        {
            if (Property *p=gazeStatePort.read(true))
            {
                Vector pose;
                p->find("depth_rgb").asList()->write(pose);
                gaze_frame_init=axis2dcm(pose.subVector(3,6));
                gaze_frame_init.setSubcol(pose.subVector(0,2),0,3);
                first_gaze_frame=false;
            }
        }

        skeletons.clear();
        if (Bottle *b=opcPort.read(false))
        {
            if (!b->get(1).isString())
            {
                for (int i=1; i<b->size(); i++)
                {
                    Property prop;
                    prop.fromString(b->get(i).asList()->toString());
                    skeletons.push_back(shared_ptr<Skeleton>(skeleton_factory(prop)));
                }
            }
        }

        if (state==State::connection_trigger)
        {
            switch_to(auto_mode?State::seek:State::idle);
        }
        else if (state==State::seek)
        {
            if (seek())
            {
                activity.clear();
                switch_to(State::follow);
                first_follow_look=true;
            }
            else if (Time::now()-still_t0>T)
            {
                Vector target(2);
                if (first_seek_look)
                {
                    target=0.0;
                    first_seek_look=false;
                }
                else
                {
                    target[0]=Rand::scalar(-30.0,30.0);
                    target[1]=Rand::scalar(-5.0,5.0);
                }
                look("angular",target);
                wait_motion_done();
                still_t0=Time::now();
            }
        }
        else if (state==State::follow)
        {
            // raised hand preemption in auto mode
            if (auto_mode)
            {
                if (auto s=find_skeleton_raised_hand())
                {
                    tag=s->getTag();
                    yInfo()<<"[Auto]: found raised hand => follow"<<tag;
                }
            }

            if (auto s=find_skeleton_tag(tag))
            {
                // pick up the requested keypoint if visible,
                // otherwise go with the first visible keypoint
                string keypoint=((*s)[this->keypoint]?this->keypoint:KeyPointTag::head);
                if (!(*s)[keypoint]->isUpdated())
                {
                    for (unsigned int i=0; i<s->getNumKeyPoints(); i++)
                    {
                        if ((*s)[i]->isUpdated())
                        {
                            keypoint=(*s)[i]->getTag();
                            break;
                        }
                    }
                }

                Vector x=(*s)[keypoint]->getPoint();
                x.push_back(1.0);
                if(virtual_mode)
                {
                    x=gaze_frame_init*x;
                    look("cartesian",x);
                }
                else
                {
                    x=gaze_frame*x;
                    look("image",(*s)[keypoint]->getPixel());
                }
                x.pop_back();
                is_following_x=x;

                // gaze speed is faster when chasing,
                // hence wait till the first movement is done
                // since it may be causing image blur
                if (first_follow_look)
                {
                    wait_motion_done();
                    first_follow_look=false;
                }

                // switch attention in auto mode
                // if the skeleton is inactive
                if (auto_mode)
                {
                    if (is_inactive(s))
                    {
                        yInfo()<<"[Auto]: detected inactivity => seek mode";
                        switch_to(State::seek);
                    }
                }

                lost_t0=Time::now();
            }
            else if (Time::now()-lost_t0>=T)
            {
                yInfo()<<"Lost track => seek mode";
                switch_to(State::seek);
            }
        }

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        // homing gaze
        look("angular",zeros(2));

        opcPort.close();
        gazeCmdPort.close();
        gazeStatePort.close();
        cmdPort.close();
        return true;
    }

public:
    /****************************************************************/
    Attention() : reporter(*this) { }
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
    rf.configure(argc,argv);

    Attention attention;
    return attention.runModule(rf);
}


