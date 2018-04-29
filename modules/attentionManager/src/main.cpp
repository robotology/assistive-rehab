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
    enum class State { idle, seek, follow } state;
    bool auto_mode;

    const int ack=Vocab::encode("ack");
    const double T=3.0;
    double inactivity_thres;
    double still_t0;
    double lost_t0;
    bool first_follow_look;

    Matrix gaze_frame;
    vector<shared_ptr<Skeleton>> skeletons;
    vector<double> activity;
    string tag;
    string keypoint;

    Mutex mutex;
    BufferedPort<Bottle> opcPort;
    RpcClient gazeCmdPort;
    BufferedPort<Property> gazeStatePort;
    RpcServer cmdPort;

    /****************************************************************/
    bool look(const string &tag, const string &keypoint) override
    {
        LockGuard lg(mutex);
        this->tag=tag;
        this->keypoint=keypoint;
        state=State::seek;
        return set_gaze_T(2.0);
    }

    /****************************************************************/
    bool stop() override
    {
        LockGuard lg(mutex);
        Bottle cmd,rep;
        cmd.addVocab(Vocab::encode("stop"));
        state=State::idle;
        if (gazeCmdPort.write(cmd,rep))
        {
            return (rep.get(0).asVocab()==ack);
        }
        return false;
    }

    /****************************************************************/
    bool is_running() override
    {
        LockGuard lg(mutex);
        return (state!=State::idle);
    }

    /****************************************************************/
    bool set_auto() override
    {
        LockGuard lg(mutex);
        auto_mode=true;
        state=State::seek;
        return set_gaze_T(2.0);
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
    shared_ptr<Skeleton> find_skeleton_raised_hand() const
    {
        for (auto &s:skeletons)
        {
            if (!(*s)[KeyPointTag::shoulder_center]->isUpdated())
            {
                continue;
            }

            bool left_raised=false;
            if ((*s)[KeyPointTag::hand_left]->isUpdated())
            {
                left_raised=((*s)[KeyPointTag::hand_left]->getPoint()[1]<
                             (*s)[KeyPointTag::shoulder_center]->getPoint()[1]);
            }

            bool right_raised=false;
            if ((*s)[KeyPointTag::hand_right]->isUpdated())
            {
                right_raised=((*s)[KeyPointTag::hand_right]->getPoint()[1]<
                              (*s)[KeyPointTag::shoulder_center]->getPoint()[1]);
            }

            if (left_raised || right_raised)
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
            yInfo()<<"Inactivity detection of"<<s->getTag()
                   <<":"<<stdev<<"/"<<inactivity_thres;
            return (stdev<inactivity_thres);
        }
        else
        {
            return false;
        }
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
            yInfo()<<"[Auto]: follow"<<tag;
            keypoint=KeyPointTag::head;
            return !skeletons.empty();
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
        loc.addList().read(const_cast<Vector&>(v));

        Property options;
        options.put("control-frame","depth_rgb");
        options.put("target-type",type);
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
        inactivity_thres=rf.check("inactivity-thres",Value(0.05)).asDouble();
        state=(auto_mode?State::seek:State::idle);

        opcPort.open("/attentionManager/opc:i");
        gazeCmdPort.open("/attentionManager/gaze/cmd:rpc");
        gazeStatePort.open("/attentionManager/gaze/state:i");
        cmdPort.open("/attentionManager/cmd:rpc");
        attach(cmdPort);

        still_t0=lost_t0=Time::now();
        first_follow_look=false;

        Rand::init();
        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return 0.1;
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

        if (state==State::seek)
        {
            if (seek())
            {
                activity.clear();
                set_gaze_T(1.0);
                state=State::follow;
                first_follow_look=true;
            }
            else if (Time::now()-still_t0>T)
            {
                Vector random(2);
                random[0]=Rand::scalar(-30.0,30.0);
                random[1]=Rand::scalar(-10.0,20.0);
                look("angular",random);
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
                x=gaze_frame*x;
                x.pop_back();
                look("cartesian",x);

                // gaze speed is faster when chasing,
                // hence wait till the first movement is done
                // since it may be wide causing image blur
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
                        set_gaze_T(2.0);
                        state=State::seek;
                    }
                }

                lost_t0=Time::now();
            }
            else if (Time::now()-lost_t0>=T)
            {
                yInfo()<<"Lost track => seek mode";
                set_gaze_T(2.0);
                state=State::seek;
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

