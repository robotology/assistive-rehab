/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/Mutex.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include <cer_kinematics/head.h>
#include <cer_kinematics/arm.h>

#include "AssistiveRehab/skeleton.h"
#include "src/robotSkeletonPublisher_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace cer::kinematics;
using namespace assistive_rehab;


/******************************************************************************/
struct PortHandler
{
    BufferedPort<Vector> port;
    Vector encs;
    double stamp;
};


/******************************************************************************/
class Publisher : public RFModule, public robotSkeletonPublisher_IDL
{
    vector<shared_ptr<PortHandler>> phdl_encs;
    vector<shared_ptr<PortHandler>> phdl_head;
    vector<shared_ptr<PortHandler>> phdl_left_arm;
    vector<shared_ptr<PortHandler>> phdl_right_arm;
    double period;

    BufferedPort<Bottle> viewerPort;
    string skeleton_name;
    vector<double> skeleton_color;
    Bottle b_sk_color;

    RpcClient opcPort;
    int opc_id;

    RpcServer cmdPort;
    Mutex mutex;
    bool visibility;

    HeadSolver head;
    ArmSolver  left_arm;
    ArmSolver  right_arm;

    /**************************************************************************/
    bool openPort(const string& robot, const string& part,
                  vector<shared_ptr<PortHandler>> &handlers)
    {
        shared_ptr<PortHandler> handler(new PortHandler);
        string remote_name="/"+robot+"/"+part+"/state:o";
        handler->port.open("/robotSkeletonPublisher/"+part+"/state:i");
        if (!Network::connect(remote_name,handler->port.getName(),"udp"))
        {
            yError()<<"Unable to connect to"<<remote_name;
            return false;
        }
        handlers.push_back(handler);
        return true;
    }

    /**************************************************************************/
    bool readHandler(shared_ptr<PortHandler> handler)
    {
        bool wait=(handler->encs.length()==0);
        Vector *v=handler->port.read(wait);
        if (v)
        {
            Stamp stamp_;
            handler->port.getEnvelope(stamp_);
            handler->encs=*v;
            handler->stamp=stamp_.getTime();
        }
        return (!wait || v);
    }

    /**************************************************************************/
    Vector getHeadEncs(const vector<shared_ptr<PortHandler>> &handlers)
    {
        Vector encs(6,0.0);
        encs.setSubvector(0,handlers[0]->encs.subVector(0,2));
        encs[3]=handlers[1]->encs[3];
        encs.setSubvector(4,handlers[2]->encs.subVector(0,1));
        return encs;
    }

    /**************************************************************************/
    Vector getArmEncs(const vector<shared_ptr<PortHandler>> &handlers)
    {
        Vector encs(12, 0.0);
        encs.setSubvector(0,handlers[0]->encs.subVector(0,2));
        encs[3]=handlers[1]->encs[3];
        encs.setSubvector(4,handlers[2]->encs.subVector(0,4));
        encs.setSubvector(9,handlers[3]->encs.subVector(0,2));
        return encs;
    }

    /**************************************************************************/
    bool set_visibility(const bool flag) override
    {
        LockGuard lg(mutex);
        visibility=flag;
        return true;
    }

    /**************************************************************************/
    bool set_robot_skeleton_name(const string &skeleton_name) override
    {
        LockGuard lg(mutex);
        this->skeleton_name=skeleton_name;
        return true;
    }

    /**************************************************************************/
    void viewerUpdate(shared_ptr<SkeletonStd> skeleton)
    {
        if (visibility && (viewerPort.getOutputCount()>0))
        {
            Bottle payLoad;
            payLoad.read(skeleton->toProperty());
            payLoad.append(b_sk_color);

            Bottle &msg=viewerPort.prepare();
            msg.clear();
            msg.addList()=payLoad;
            viewerPort.writeStrict();
        }
    }

    /**************************************************************************/
    bool opcUpdate(shared_ptr<SkeletonStd> skeleton, const double stamp)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            if (opc_id<0)
            {
                cmd.addVocab(Vocab::encode("add"));
                Property prop=skeleton->toProperty();
                prop.put("stamp",stamp);
                cmd.addList().read(prop);
                if (opcPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==Vocab::encode("ack"))
                    {
                        opc_id=rep.get(1).asList()->get(1).asInt();
                        return true;
                    }
                }
            }
            else
            {
                cmd.addVocab(Vocab::encode("set"));
                Bottle &pl=cmd.addList();
                Property prop=skeleton->toProperty();
                prop.put("stamp",stamp);
                pl.read(prop);
                Bottle id;
                Bottle &id_pl=id.addList();
                id_pl.addString("id");
                id_pl.addInt(opc_id);
                pl.append(id);
                if (opcPort.write(cmd,rep))
                {
                    return (rep.get(0).asVocab()==Vocab::encode("ack"));
                }
            }
        }
        return false;
    }

    /**************************************************************************/
    bool opcDel()
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("del"));
            Bottle &pl=cmd.addList().addList();
            pl.addString("id");
            pl.addInt(opc_id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab()==Vocab::encode("ack"));
            }
        }
        return false;
    }

    /**************************************************************************/
    bool attach(RpcServer &source) override
    {
        return yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        string robot=rf.check("robot",Value("cer")).asString();
        skeleton_name=rf.check("skeleton-name",Value("robot")).asString();
        period=rf.check("period",Value(0.05)).asDouble();
        visibility=rf.check("visibility",Value(true)).asBool();

        skeleton_color={0.23,0.7,0.44};
        if (rf.check("skeleton-color"))
        {
            if (const Bottle *ptr=rf.find("skeleton-color").asList())
            {
                size_t len=std::min(skeleton_color.size(),ptr->size());
                for (size_t i=0; i<len; i++)
                {
                    skeleton_color[i]=ptr->get(i).asDouble();
                }
            }
        }
        Bottle &b1=b_sk_color.addList();
        b1.addString("color");
        Bottle &b2=b1.addList();
        for (auto &c:skeleton_color)
        {
            b2.addDouble(c);
        }

        if (!openPort(robot,"torso_tripod",phdl_encs)      ||
            !openPort(robot,"torso",phdl_encs)             ||
            !openPort(robot,"head",phdl_encs)              ||
            !openPort(robot,"left_arm",phdl_encs)          ||
            !openPort(robot,"left_wrist_tripod",phdl_encs) ||
            !openPort(robot,"right_arm",phdl_encs)         ||
            !openPort(robot,"right_wrist_tripod",phdl_encs))
        {
            close();
        }

        phdl_head.push_back(phdl_encs[0]);
        phdl_head.push_back(phdl_encs[1]);
        phdl_head.push_back(phdl_encs[2]);

        phdl_left_arm.push_back(phdl_encs[0]);
        phdl_left_arm.push_back(phdl_encs[1]);
        phdl_left_arm.push_back(phdl_encs[3]);
        phdl_left_arm.push_back(phdl_encs[4]);

        phdl_right_arm.push_back(phdl_encs[0]);
        phdl_right_arm.push_back(phdl_encs[1]);
        phdl_right_arm.push_back(phdl_encs[5]);
        phdl_right_arm.push_back(phdl_encs[6]);

        viewerPort.open("/robotSkeletonPublisher/viewer:o");
        opcPort.open("/robotSkeletonPublisher/opc:rpc");
        opc_id=-1;

        cmdPort.open("/robotSkeletonPublisher/cmd:rpc");
        attach(cmdPort);

        HeadParameters params_head("depth_center");
        params_head.head.setAllConstraints(false);
        head.setHeadParameters(params_head);

        ArmParameters params_left_arm("left");
        params_left_arm.upper_arm.setAllConstraints(false);
        left_arm.setArmParameters(params_left_arm);

        ArmParameters params_right_arm("right");
        params_right_arm.upper_arm.setAllConstraints(false);
        right_arm.setArmParameters(params_right_arm);

        return true;
    }

    /**************************************************************************/
    double getPeriod() override
    {
        return period;
    }

    /**************************************************************************/
    bool updateModule() override
    {
        LockGuard lg(mutex);

        // update joints state and time stamps
        Vector stamps;
        for (auto &h:phdl_encs)
        {
            if (!readHandler(h))
            {
                return false;
            }
            stamps.push_back(h->stamp);
        }
        double stamp=findMax(stamps);

        // prepare joints state for single chains
        Vector encs_head=getHeadEncs(phdl_head);
        Vector encs_left_arm=getArmEncs(phdl_left_arm);
        Vector encs_right_arm=getArmEncs(phdl_right_arm);
        
        // compute fkin + update the skeleton info:
        // root frame is "depth_center"
        Matrix root,hee;
        head.fkin(encs_head,root);
        root=SE3inv(root);

        vector<pair<string,Vector>> unordered;
        unordered.push_back(make_pair(KeyPointTag::head,Vector(3,0.0)));

        head.fkin(encs_head,hee,4); hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::shoulder_center,hee.getCol(3).subVector(0,2)));

        hee=eye(4,4); hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::hip_center,hee.getCol(3).subVector(0,2)));

        hee=eye(4,4); hee(1,3)=0.1; hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::hip_left,hee.getCol(3).subVector(0,2)));

        hee=eye(4,4); hee(1,3)=-0.1; hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::hip_right,hee.getCol(3).subVector(0,2)));

        left_arm.fkin(encs_left_arm,hee,5); hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::shoulder_left,hee.getCol(3).subVector(0,2)));

        left_arm.fkin(encs_left_arm,hee,6); hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::elbow_left,hee.getCol(3).subVector(0,2)));

        left_arm.fkin(encs_left_arm,hee); hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::hand_left,hee.getCol(3).subVector(0,2)));

        right_arm.fkin(encs_right_arm,hee,5); hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::shoulder_right,hee.getCol(3).subVector(0,2)));

        right_arm.fkin(encs_right_arm,hee,6); hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::elbow_right,hee.getCol(3).subVector(0,2)));

        right_arm.fkin(encs_right_arm,hee); hee=root*hee;
        unordered.push_back(make_pair(KeyPointTag::hand_right,hee.getCol(3).subVector(0,2)));

        shared_ptr<SkeletonStd> skeleton(new SkeletonStd());
        skeleton->setTag(skeleton_name);
        skeleton->update(unordered);

        viewerUpdate(skeleton);
        opcUpdate(skeleton,stamp);

        return true;
    }

    /**************************************************************************/
    bool interruptModule() override
    {
        for (auto &h:phdl_encs)
        {
            h->port.interrupt();
        }
        return true;
    }

    /**************************************************************************/
    bool close() override
    {
        if (cmdPort.asPort().isOpen())
        {
            cmdPort.close();
        }
        if (opcPort.asPort().isOpen())
        {
            opcDel();
            opcPort.close();
        }
        if (!viewerPort.isClosed())
        {
            viewerPort.close();
        }
        for (auto &h:phdl_encs)
        {
            if (!h->port.isClosed())
            {
                h->port.close();
            }
        }
        return true;
    }
};


/******************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("robotSkeletonPublisher");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Publisher publisher;
    return publisher.runModule(rf);
}
