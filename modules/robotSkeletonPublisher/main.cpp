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

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

#include <cer_kinematics/head.h>
#include <cer_kinematics/arm.h>

#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace cer::kinematics;
using namespace assistive_rehab;


/******************************************************************************/
class Publisher : public RFModule
{
    PolyDriver drivers[7];
    vector<IEncodersTimed*> ienc_head;
    vector<IEncodersTimed*> ienc_left_arm;
    vector<IEncodersTimed*> ienc_right_arm;
    double period;

    BufferedPort<Bottle> viewerPort;
    Bottle color;

    HeadSolver head;
    ArmSolver  left_arm;
    ArmSolver  right_arm;

    /**************************************************************************/
    bool openDriver(PolyDriver &driver, const string robot, const string part)
    {
        Property option;
        option.put("device","remote_controlboard");
        option.put("remote","/"+robot+"/"+part);
        option.put("local","/robotSkeletonPublisher/"+part);
        option.put("writeStrict","on");
        if (!driver.open(option))
        {
            yError("Unable to connect to %s",string("/"+robot+"/"+part).c_str());
            return false;
        }
        return true;
    }

    /**************************************************************************/
    Vector getHeadEncs(const vector<IEncodersTimed*> &lienc, double &stamp)
    {
        Vector encs(6,0.0);
        Vector stamps(encs.length());

        Vector encs_=encs;
        Vector stamps_=stamps;

        lienc[0]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(0,encs_.subVector(0,2));
        stamps.setSubvector(0,stamps_.subVector(0,2));

        lienc[1]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs[3]=encs_[3];
        stamps[3]=stamps_[3];

        lienc[2]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(4,encs_.subVector(0,1));
        stamps.setSubvector(4,stamps_.subVector(0,1));

        stamp=findMax(stamps);
        return encs;

    }

    /**************************************************************************/
    Vector getArmEncs(const vector<IEncodersTimed*> &lienc, double &stamp)
    {
        Vector encs(12,0.0);
        Vector stamps(encs.length());

        Vector encs_=encs;
        Vector stamps_=stamps;

        lienc[0]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(0,encs_.subVector(0,2));
        stamps.setSubvector(0,stamps_.subVector(0,2));

        lienc[1]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs[3]=encs_[3];
        stamps[3]=stamps_[3];

        lienc[2]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(4,encs_.subVector(0,4));
        stamps.setSubvector(4,stamps_.subVector(0,4));

        lienc[3]->getEncodersTimed(&encs_[0],&stamps_[0]);
        encs.setSubvector(9,encs_.subVector(0,2));
        stamps.setSubvector(9,stamps_.subVector(0,2));

        stamp=findMax(stamps);
        return encs;
    }

public:
    /**************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("cer")).asString();
        period=rf.check("period",Value(0.05)).asDouble();

        if (!openDriver(drivers[0],robot,"torso_tripod")      ||
            !openDriver(drivers[1],robot,"torso")             ||
            !openDriver(drivers[2],robot,"head")              ||
            !openDriver(drivers[3],robot,"left_arm")          ||
            !openDriver(drivers[4],robot,"left_wrist_tripod") ||
            !openDriver(drivers[5],robot,"right_arm")         ||
            !openDriver(drivers[6],robot,"right_wrist_tripod"))
        {
            close();
        }

        IEncodersTimed *ienc[7];
        for (int i=0; i<7; i++)
        {
            drivers[i].view(ienc[i]);
        }

        ienc_head.push_back(ienc[0]);
        ienc_head.push_back(ienc[1]);
        ienc_head.push_back(ienc[2]);

        ienc_left_arm.push_back(ienc[0]);
        ienc_left_arm.push_back(ienc[1]);
        ienc_left_arm.push_back(ienc[3]);
        ienc_left_arm.push_back(ienc[4]);

        ienc_right_arm.push_back(ienc[0]);
        ienc_right_arm.push_back(ienc[1]);
        ienc_right_arm.push_back(ienc[5]);
        ienc_right_arm.push_back(ienc[6]);

        viewerPort.open("/robotSkeletonPublisher/viewer:o");

        HeadParameters params_head("depth_center");
        params_head.head.setAllConstraints(false);
        head.setHeadParameters(params_head);

        ArmParameters params_left_arm("left");
        params_left_arm.upper_arm.setAllConstraints(false);
        left_arm.setArmParameters(params_left_arm);

        ArmParameters params_right_arm("right");
        params_right_arm.upper_arm.setAllConstraints(false);
        right_arm.setArmParameters(params_right_arm);

        color=Bottle("(color (0.23 0.7 0.44))");

        return true;
    }

    /**************************************************************************/
    double getPeriod()
    {
        return period;
    }

    /**************************************************************************/
    bool updateModule()
    {
        Vector stamps(3);
        Vector encs_head=getHeadEncs(ienc_head,stamps[0]);
        Vector encs_left_arm=getArmEncs(ienc_left_arm,stamps[1]);
        Vector encs_right_arm=getArmEncs(ienc_right_arm,stamps[2]);
        double stamp=findMax(stamps);

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

        SkeletonStd skeleton;
        skeleton.setTag("robot");
        skeleton.update(unordered);

        if (viewerPort.getOutputCount()>0)
        {
            Bottle msg;
            msg.read(skeleton.toProperty());
            msg.append(color);
            viewerPort.prepare().addList()=msg;
            viewerPort.writeStrict();
        }

        return true;
    }

    /**************************************************************************/
    bool close()
    {
        viewerPort.close();
        for (int i=0; i<7; i++)
        {
           if (drivers[i].isValid())
           {
               drivers[i].close();
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
