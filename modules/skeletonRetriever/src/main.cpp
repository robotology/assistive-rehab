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
#include <cmath>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <sstream>
#include <iostream>
#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/filters.h>

#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace assistive_rehab;


/****************************************************************/
class MetaSkeleton
{
    bool filter_enable;
    vector<shared_ptr<MedianFilter>> filter;

public:
    double timer;
    int opc_id;
    shared_ptr<SkeletonWaist> skeleton;
    vector<int> keys_acceptable_misses;

    /****************************************************************/
    MetaSkeleton(const double t, const bool filter_enable_,
                 const int filter_order_) :
        timer(t), filter_enable(filter_enable_), opc_id(-1)
    {
        skeleton=shared_ptr<SkeletonWaist>(new SkeletonWaist());
        keys_acceptable_misses.assign(skeleton->getNumKeyPoints(),0);

        if (filter_enable)
        {
            for (unsigned int i=0; i<skeleton->getNumKeyPoints(); i++)
            {
                filter.push_back(shared_ptr<MedianFilter>(
                   new MedianFilter(filter_order_,(*skeleton)[i]->getPoint())));
            }
        }
    }

    /****************************************************************/
    bool init(const string &tag, const Vector &p)
    {
        int i=skeleton->getNumFromKey(tag);
        if (i>=0)
        {
            filter[i]->init(p);
            return true;
        }
        else
            return false;
    }

    /****************************************************************/
    void update(const vector<pair<string,Vector>> &unordered)
    {
        if (filter_enable)
        {
            vector<pair<string,Vector>> unordered_fitered;
            for (auto &p:unordered)
            {
                int i=skeleton->getNumFromKey(p.first);
                if (i>=0)
                {
                    unordered_fitered.push_back(make_pair(p.first,filter[i]->filt(p.second)));
                }
            }
            skeleton->update(unordered_fitered);
        }
        else
            skeleton->update(unordered);
    }
};


/****************************************************************/
class Retriever : public RFModule
{
    BufferedPort<Bottle> skeletonsPort;
    BufferedPort<ImageOf<PixelFloat>> depthPort;
    BufferedPort<Bottle> viewerPort;
    RpcClient opcPort;
    RpcClient camPort;

    ImageOf<PixelFloat> depth;

    unordered_map<string,string> keysRemap;
    vector<MetaSkeleton> skeletons;

    bool camera_configured;
    double period;
    double fov_h;
    double fov_v;
    double keys_recognition_confidence;
    double keys_recognition_percentage;
    int keys_acceptable_misses;
    double tracking_threshold;
    double time_to_live;
    bool filter_enable;
    int filter_order;
    double t0;

    /****************************************************************/
    bool getCameraOptions()
    {
        if (camPort.getOutputCount()>0)
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("visr"));
            cmd.addVocab(Vocab::encode("get"));
            cmd.addVocab(Vocab::encode("fov"));
            if (camPort.write(cmd,rep))
            {
                if (rep.size()>=5)
                {
                    fov_h=rep.get(3).asDouble();
                    fov_v=rep.get(4).asDouble();
                    yInfo()<<"retrieved from camera fov_h="<<fov_h;
                    yInfo()<<"retrieved from camera fov_v="<<fov_v;
                    return true;
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool getPoint3D(const int u, const int v, Vector &p) const
    {
        double f=depth.width()/(2.0*tan(fov_h*(M_PI/180.0)/2.0));
        double d=depth(u,v);
        if ((d>0.0) && (f>0.0))
        {
            double x=u-0.5*(depth.width()-1);
            double y=v-0.5*(depth.height()-1);

            p=d*ones(3);
            p[0]*=x/f;
            p[1]*=y/f;

            return true;
        }
        else
        {
            return false;
        }
    }

    /****************************************************************/
    MetaSkeleton create(Bottle *keys)
    {
        MetaSkeleton s(time_to_live,filter_enable,filter_order);
        vector<pair<string,Vector>> unordered;
        vector<Vector> hips;

        Vector p;
        for (int i=0; i<keys->size(); i++)
        {
            if (Bottle *k=keys->get(i).asList())
            {
                if (k->size()>=4)
                {
                    string tag=k->get(0).asString();
                    int u=(int)k->get(1).asDouble();
                    int v=(int)k->get(2).asDouble();
                    double confidence=k->get(3).asDouble();

                    if ((confidence>=keys_recognition_confidence) && getPoint3D(u,v,p))
                    {
                        unordered.push_back(make_pair(keysRemap[tag],p));
                        if ((keysRemap[tag]==KeyPointTag::hip_left) ||
                            (keysRemap[tag]==KeyPointTag::hip_right))
                        {
                            hips.push_back(p);
                        }
                    }
                }
            }
        }

        if (hips.size()==2)
        {
            unordered.push_back(make_pair(KeyPointTag::hip_center,0.5*(hips[0]+hips[1])));
        }

        s.skeleton->update(unordered);
        return s;
    }

    /****************************************************************/
    void update(const MetaSkeleton &src, MetaSkeleton &dest)
    {
        vector<pair<string,Vector>> unordered;
        for (unsigned int i=0; i<src.skeleton->getNumKeyPoints(); i++)
        {
            auto key=(*src.skeleton)[i];
            if (key->isUpdated())
            {
                const Vector &p=key->getPoint();
                unordered.push_back(make_pair(key->getTag(),p));
                dest.keys_acceptable_misses[i]=keys_acceptable_misses;
                dest.init(key->getTag(),p);
            }
            else if (dest.keys_acceptable_misses[i]>0)
            {
                unordered.push_back(make_pair(key->getTag(),(*dest.skeleton)[i]->getPoint()));
                dest.keys_acceptable_misses[i]--;
            }
        }

        dest.update(unordered);
        dest.timer=time_to_live;
    }

    /****************************************************************/
    bool isValid(const MetaSkeleton &s) const
    {
        unsigned int n=0;
        for (unsigned int i=0; i<s.skeleton->getNumKeyPoints(); i++)
        {
            if ((*s.skeleton)[i]->isUpdated())
            {
                n++;
            }
        }
        
        double perc=((double)n)/((double)s.skeleton->getNumKeyPoints());
        return (perc>=keys_recognition_percentage);
    }

    /****************************************************************/
    MetaSkeleton *isTracked(const MetaSkeleton &s)
    {
        map<double,int> scores;
        for (unsigned int i=0; i<skeletons.size(); i++)
        {
            double mean=0.0; int num=0;
            auto &sk=*(skeletons[i].skeleton);
            for (unsigned int j=0; j<sk.getNumKeyPoints(); j++)
            {
                if (sk[j]->isUpdated() && (*s.skeleton)[j]->isUpdated())
                {
                    mean+=norm(sk[j]->getPoint()-(*s.skeleton)[j]->getPoint());
                }
                num++;
            }
            if (num>0)
            {
                mean/=num;
                if (mean<=tracking_threshold)
                {
                    scores[mean]=i;
                }
            }
        }

        return (scores.empty()?nullptr:&skeletons[scores.begin()->second]);
    }

    /****************************************************************/
    bool opcAdd(MetaSkeleton &s)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("add"));
            Property prop=s.skeleton->toProperty();
            cmd.addList().read(prop);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    s.opc_id=rep.get(1).asList()->get(1).asInt();
                    ostringstream ss;
                    ss<<"#"<<hex<<s.opc_id;
                    s.skeleton->setTag(ss.str());
                    return opcSet(s);
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcSet(const MetaSkeleton &s)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("set"));
            Bottle &pl=cmd.addList();
            Property prop=s.skeleton->toProperty();
            pl.read(prop);
            Bottle id;
            Bottle &id_pl=id.addList();
            id_pl.addString("id");
            id_pl.addInt(s.opc_id);
            pl.append(id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab()==Vocab::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcDel(const MetaSkeleton &s)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("del"));
            Bottle &pl=cmd.addList().addList();
            pl.addString("id");
            pl.addInt(s.opc_id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab()==Vocab::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    void gc(const double dt)
    {
        vector<MetaSkeleton> skeletons_;
        for (auto &s:skeletons)
        {
            s.timer-=dt;
            if (s.timer>0.0)
            {
                skeletons_.push_back(s);
            }
            else
            {
                opcDel(s);
            }
        }

        skeletons=skeletons_;
    }

    /****************************************************************/
    void viewerUpdate()
    {
        if (viewerPort.getOutputCount()>0)
        {
            Bottle &msg=viewerPort.prepare();
            msg.clear();
            for (auto &s:skeletons)
            {
                Property prop=s.skeleton->toProperty();
                msg.addList().read(prop);
            }
            viewerPort.writeStrict();
        }
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        keysRemap["Nose"]=KeyPointTag::head;
        keysRemap["Neck"]=KeyPointTag::shoulder_center;
        keysRemap["RShoulder"]=KeyPointTag::shoulder_right;
        keysRemap["RElbow"]=KeyPointTag::elbow_right;
        keysRemap["RWrist"]=KeyPointTag::hand_right;
        keysRemap["LShoulder"]=KeyPointTag::shoulder_left;
        keysRemap["LElbow"]=KeyPointTag::elbow_left;
        keysRemap["LWrist"]=KeyPointTag::hand_left;
        keysRemap["RHip"]=KeyPointTag::hip_right;
        keysRemap["RKnee"]=KeyPointTag::knee_right;
        keysRemap["RAnkle"]=KeyPointTag::ankle_right;
        keysRemap["LHip"]=KeyPointTag::hip_left;
        keysRemap["LKnee"]=KeyPointTag::knee_left;
        keysRemap["LAnkle"]=KeyPointTag::ankle_left;

        // default values
        period=0.01;
        keys_recognition_confidence=0.3;
        keys_recognition_percentage=0.3;
        keys_acceptable_misses=3;
        tracking_threshold=0.4;
        time_to_live=1.0;
        filter_enable=true;
        filter_order=5;

        // retrieve values from config file
        Bottle &gGeneral=rf.findGroup("general");
        if (!gGeneral.isNull())
        {
            period=gGeneral.check("period",period).asDouble();
        }

        Bottle &gSkeleton=rf.findGroup("skeleton");
        if (!gSkeleton.isNull())
        {
            keys_recognition_confidence=gSkeleton.check("key-recognition-confidence",keys_recognition_confidence).asDouble();
            keys_recognition_percentage=gSkeleton.check("key-recognition-percentage",keys_recognition_percentage).asDouble();
            keys_acceptable_misses=gSkeleton.check("keys-acceptable-misses",keys_acceptable_misses).asInt();
            tracking_threshold=gSkeleton.check("tracking-threshold",tracking_threshold).asDouble();
            time_to_live=gSkeleton.check("time-to-live",time_to_live).asDouble();
        }

        Bottle &gFiltering=rf.findGroup("filtering");
        if (!gFiltering.isNull())
        {
            filter_enable=(gFiltering.check("filter-enable",Value(filter_enable?"on":"off")).asString()=="on");
            filter_order=gFiltering.check("filter-order",filter_order).asInt();
        }

        skeletonsPort.open("/skeletonRetriever/skeletons:i");
        depthPort.open("/skeletonRetriever/depth:i");
        viewerPort.open("/skeletonRetriever/viewer:o");
        opcPort.open("/skeletonRetriever/opc:rpc");
        camPort.open("/skeletonRetriever/cam:rpc");

        camera_configured=false;
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
        const double t=Time::now();
        const double dt=t-t0;
        t0=t;

        if (ImageOf<PixelFloat> *depth=depthPort.read(false))
        {
            this->depth=*depth;
        }

        if (!camera_configured)
        {
            camera_configured=getCameraOptions();
        }

        // garbage collector
        gc(dt);

        // handle skeletons acquired from detector
        if (Bottle *b1=skeletonsPort.read(false))
        {
            if (Bottle *b2=b1->get(0).asList())
            {
                bool doViewerUpdate=false;
                for (int i=0; i<b2->size(); i++)
                {
                    Bottle *b3=b2->get(i).asList();
                    if ((depth.width()>0) && (depth.height()>0) && (b3!=nullptr))
                    {
                        MetaSkeleton s=create(b3);
                        if (MetaSkeleton *s1=isTracked(s))
                        {
                            update(s,*s1);
                            opcSet(*s1);
                            doViewerUpdate=true;
                        }
                        else if (isValid(s))
                        {
                            opcAdd(s);
                            skeletons.push_back(s);
                            doViewerUpdate=true;
                        }
                    }
                }

                if (doViewerUpdate)
                {
                    viewerUpdate();
                }
            }
        }

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        skeletonsPort.close();
        depthPort.close();
        viewerPort.close();
        opcPort.close();
        camPort.close();
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
    rf.setDefaultContext("skeletonRetriever");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Retriever retriever;
    return retriever.runModule(rf);
}

