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

#include <limits>
#include <algorithm>
#include <math.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/RpcClient.h>

#include <yarp/sig/Vector.h>

#include <yarp/math/Math.h>

#include "AssistiveRehab/skeleton.h"
#include <src/skeletonLocker_IDL.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

/****************************************************************/
class MetaSkeleton
{

public:
    const int opc_id_invalid=-1;
    int opc_id;
    vector<Vector> pivots;
    string tag;
    shared_ptr<SkeletonStd> skeleton;
    double radius;
    double timer;
    double score;

    /****************************************************************/
    MetaSkeleton()
    {
        opc_id=opc_id_invalid;
        skeleton=shared_ptr<SkeletonStd>(new SkeletonStd());
        pivots.assign(2,numeric_limits<double>::infinity()*ones(3));
        score=numeric_limits<double>::infinity();
        radius=0.0;
    }

};

/****************************************************************/
class Locker : public RFModule , public skeletonLocker_IDL
{
    // ports
    BufferedPort<Bottle> opcInPort;
    BufferedPort<Bottle> viewerPort;
    RpcClient opcRpcPort;
    RpcServer rpcPort;
    RpcClient retrieverPort;

    // params
    string moduleName;
    double period;
    double radius;
    double time_trend;
    double max_radius;

    MetaSkeleton* locked;
    MetaSkeleton l;
    vector<MetaSkeleton> new_skeletons;
    string skeleton_tag;
    double t0,t;
    int opc_id_locked;
    bool added;
    string tag_locked;

    mutex mtx;

    /****************************************************************/
    bool attach(RpcServer &source) override
    {
        return yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool set_skeleton_tag(const string &tag) override
    {
        lock_guard<mutex> lg(mtx);
        if (locked->opc_id>=0)
            opcDel(*locked);

        added=false;
        skeleton_tag=tag;
        tag_locked=skeleton_tag+"-locked";
        return true;
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        moduleName=rf.check("name",Value("skeletonLocker")).asString();
        period=rf.check("period",Value(0.01)).asDouble();
        radius=rf.check("radius",Value(0.6)).asDouble();
        time_trend=rf.check("time-trend",Value(30.0)).asDouble();
        max_radius=rf.check("max-radius",Value(0.75)).asDouble();

        opcInPort.open("/"+moduleName+"/opc:i");
        viewerPort.open("/"+moduleName+"/viewer:o");
        opcRpcPort.open("/"+moduleName+"/opc:rpc");
        retrieverPort.open("/"+moduleName+"/retriever:rpc");
        rpcPort.open("/"+moduleName+"/rpc");
        attach(rpcPort);

        locked=new MetaSkeleton();
        t=t0=Time::now();
        added=false;

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
        lock_guard<mutex> lg(mtx);

        if (Bottle *b=opcInPort.read(false))
        {
            new_skeletons.clear();
            // we start from 1 because in 0 we have "sync"
            if (!b->get(1).isString())
            {
                for (int i=1; i<b->size(); i++)
                {
                    Property prop;
                    prop.fromString(b->get(i).asList()->toString());
                    string tag=prop.find("tag").asString();
                    if (!tag.empty() && tag!="robot" && !prop.check("finish-line") && !prop.check("start-line"))
                    {
                        if ( tag!=skeleton_tag && tag!=tag_locked )
                        {
                            MetaSkeleton s;
                            int id=prop.find("id").asInt();
                            s.tag=tag;
                            s.opc_id=id;
                            s.skeleton->update(prop);
                            if ( (*s.skeleton)[KeyPointTag::shoulder_center]->isUpdated() )
                            {
                                s.pivots[0]=(*s.skeleton)[KeyPointTag::shoulder_center]->getPoint();
                            }
                            if ( (*s.skeleton)[KeyPointTag::hip_center]->isUpdated() )
                            {
                                s.pivots[1]=(*s.skeleton)[KeyPointTag::hip_center]->getPoint();
                            }
                            new_skeletons.push_back(s);
                        }
                        else if ( tag==skeleton_tag )
                        {
                            int id=prop.find("id").asInt();
                            l.skeleton->update(prop);
                            l.tag=tag_locked;
                            l.skeleton->setTag(l.tag);
                            if (!added)
                                l.opc_id=id;
                            else
                                l.opc_id=opc_id_locked;
                            t=Time::now();
                            if ( (*l.skeleton)[KeyPointTag::shoulder_center]->isUpdated() )
                            {
                                l.pivots[0]=(*l.skeleton)[KeyPointTag::shoulder_center]->getPoint();
                            }
                            if ( (*l.skeleton)[KeyPointTag::hip_center]->isUpdated() )
                            {
                                l.pivots[1]=(*l.skeleton)[KeyPointTag::hip_center]->getPoint();
                            }
                        }
                    }
                }
            }
        }

        // update existing skeleton
        if (!new_skeletons.empty())
        {
            Vector scores=check_3D_consistency(*locked,new_skeletons);
            auto it=min_element(scores.begin(),scores.end());
            if (it!=scores.end())
            {
                if (*it<numeric_limits<double>::infinity())
                {
                    auto i=distance(scores.begin(),it);
                    auto &sk=new_skeletons[i];
                    yInfo()<<"Merging"<<sk.opc_id<<sk.tag;
                    locked->skeleton->update(sk.skeleton->toProperty());
                    if ( (*sk.skeleton)[KeyPointTag::shoulder_center]->isUpdated() )
                    {
                        locked->pivots[0]=(*sk.skeleton)[KeyPointTag::shoulder_center]->getPoint();
                    }
                    if ( (*l.skeleton)[KeyPointTag::hip_center]->isUpdated() )
                    {
                        locked->pivots[1]=(*sk.skeleton)[KeyPointTag::hip_center]->getPoint();
                    }
                    locked->skeleton->setTag(tag_locked);
                    locked->opc_id=opc_id_locked;
                    t=Time::now();
                    opcSet(*locked);
                }
            }
            cout<<"Skeletons: "<<endl;
            for (int i=0; i<new_skeletons.size(); i++)
            {
                cout<<new_skeletons[i].skeleton->getTag()<<" score: "<<new_skeletons[i].score<<" locked skeleton radius: "<<locked->radius<<endl;
            }
            cout<<endl;
        }

        locked=&l;
        if (locked->opc_id>=0 && !added)
        {
            added=opcAdd(*locked);
        }
        locked->timer=t-t0;
        locked->radius=radius;
        if (locked->timer<0.0)
        {
            double r=radius*(exp(-locked->timer/time_trend));
            locked->radius=(r<=max_radius?r:max_radius);
        }
        t0=Time::now();

        viewerUpdate();

        return true;
    }

    /****************************************************************/
    void viewerUpdate()
    {
        if (viewerPort.getOutputCount()>0)
        {
            Bottle &msg=viewerPort.prepare();
            msg.clear();
            Property prop=(locked->skeleton)->toProperty();
            msg.addList().read(prop);
            viewerPort.writeStrict();
        }
    }

    /****************************************************************/
    bool opcAdd(MetaSkeleton &s)
    {
        if (opcRpcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("add"));
            Property prop=(s.skeleton)->toProperty();
            cmd.addList().read(prop);
            if (opcRpcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    s.opc_id=rep.get(1).asList()->get(1).asInt();
                    opc_id_locked=s.opc_id;
                    yInfo()<<"Adding to opc"<<s.opc_id<<s.tag;
                    return opcSet(s);
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcSet(const MetaSkeleton &s)
    {
        if (opcRpcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("set"));
            Bottle &pl=cmd.addList();
            Property prop=(s.skeleton)->toProperty();
            pl.read(prop);
            Bottle id;
            Bottle &id_pl=id.addList();
            id_pl.addString("id");
            id_pl.addInt(s.opc_id);
            pl.append(id);
            if (opcRpcPort.write(cmd,rep))
            {
                yInfo()<<"Setting to opc"<<s.tag<<s.opc_id;
                return (rep.get(0).asVocab()==Vocab::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcDel(const MetaSkeleton &s)
    {
        if (opcRpcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("del"));
            Bottle &pl=cmd.addList().addList();
            pl.addString("id");
            pl.addInt(s.opc_id);
            if (opcRpcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab()==Vocab::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    Vector check_3D_consistency(const MetaSkeleton &c, vector<MetaSkeleton> &new_skels)
    {
        Vector scores;
        for ( auto &n:new_skels )
        {
            double dist=numeric_limits<double>::infinity();
            for (size_t i=0; i<n.pivots.size(); i++)
            {
                dist=std::min(dist,norm(c.pivots[i]-n.pivots[i]));
            }
            n.score=dist;
            scores.push_back(dist<=c.radius?dist:numeric_limits<double>::infinity());
        }

        return scores;
    }

    /****************************************************************/
    bool close() override
    {
        if (opcDel(*locked))
            yInfo()<<"Removed skeleton locked from opc";

        yInfo()<<"Deleting memory";
        locked=nullptr;
        delete locked;

        yInfo()<<"Closing";
        opcInPort.close();
        viewerPort.close();
        opcRpcPort.close();
        retrieverPort.close();
        rpcPort.close();
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
    rf.setDefaultContext("skeletonLocker");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Locker locker;
    return locker.runModule(rf);
}

