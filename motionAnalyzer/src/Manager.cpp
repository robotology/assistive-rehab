#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <Manager.h>
#include <Processor.h>
#include <Metric.h>

#include "motionAnalyzer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

void Manager::init()
{
    //joints initial configuration
    elbowLeft_init.resize(3);
    elbowRight_init.resize(3);
    handLeft_init.resize(3);
    handRight_init.resize(3);
    head_init.resize(3);
    shoulderCenter_init.resize(3);
    shoulderLeft_init.resize(3);
    shoulderRight_init.resize(3);
    hipLeft_init.resize(3);
    hipRight_init.resize(3);
    kneeLeft_init.resize(3);
    kneeRight_init.resize(3);

    //joint vectors
    //index = 3, value = 0/1              => stationary/mobile joint
    //index = 4, value = 0/max azimuth    => stationary/mobile joint
    //index = 5, value = 0/max elevation  => stationary/mobile joint
    elbowLeft.resize(6);
    elbowRight.resize(6);
    handLeft.resize(6);
    handRight.resize(6);
    head.resize(6);
    shoulderCenter.resize(6);
    shoulderLeft.resize(6);
    shoulderRight.resize(6);
    hipLeft.resize(6);
    hipRight.resize(6);
    kneeLeft.resize(6);
    kneeRight.resize(6);

}

bool Manager::loadInitialConf()
{
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext(this->rf->getContext().c_str());
    rf.setDefaultConfigFile(this->rf->find("configuration-file").asString().c_str());
    rf.configure(0, NULL);

    Bottle &bGeneral = rf.findGroup("GENERAL");

    if(!bGeneral.isNull())
    {
        nmovements = bGeneral.find("number_movements").asInt();

        if(Bottle *bElbowLeft_init = bGeneral.find("elbow_left_init_pose").asList())
        {
            elbowLeft_init[0] = bElbowLeft_init->get(0).asDouble();
            elbowLeft_init[1] = bElbowLeft_init->get(1).asDouble();
            elbowLeft_init[2] = bElbowLeft_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for elbow left";

        if(Bottle *bElbowRight_init = bGeneral.find("elbow_right_init_pose").asList())
        {
            elbowRight_init[0] = bElbowRight_init->get(0).asDouble();
            elbowRight_init[1] = bElbowRight_init->get(1).asDouble();
            elbowRight_init[2] = bElbowRight_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for elbow right";

        if(Bottle *bHandLeft_init = bGeneral.find("hand_left_init_pose").asList())
        {
            handLeft_init[0] = bHandLeft_init->get(0).asDouble();
            handLeft_init[1] = bHandLeft_init->get(1).asDouble();
            handLeft_init[2] = bHandLeft_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for hand left";

        if(Bottle *bHandRight_init = bGeneral.find("hand_right_init_pose").asList())
        {
            handRight_init[0] = bHandRight_init->get(0).asDouble();
            handRight_init[1] = bHandRight_init->get(1).asDouble();
            handRight_init[2] = bHandRight_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for hand right";

        if(Bottle *bHead_init = bGeneral.find("head_init_pose").asList())
        {
            head_init[0] = bHead_init->get(0).asDouble();
            head_init[1] = bHead_init->get(1).asDouble();
            head_init[2] = bHead_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for hand left";

        if(Bottle *bShoulderCenter_init = bGeneral.find("shoulder_center_init_pose").asList())
        {
            shoulderCenter_init[0] = bShoulderCenter_init->get(0).asDouble();
            shoulderCenter_init[1] = bShoulderCenter_init->get(1).asDouble();
            shoulderCenter_init[2] = bShoulderCenter_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for shoulder center";

        if(Bottle *bShoulderLeft_init = bGeneral.find("shoulder_left_init_pose").asList())
        {
            shoulderLeft_init[0] = bShoulderLeft_init->get(0).asDouble();
            shoulderLeft_init[1] = bShoulderLeft_init->get(1).asDouble();
            shoulderLeft_init[2] = bShoulderLeft_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for shoulder left";

        if(Bottle *bShoulderRight_init = bGeneral.find("shoulder_right_init_pose").asList())
        {
            shoulderRight_init[0] = bShoulderRight_init->get(0).asDouble();
            shoulderRight_init[1] = bShoulderRight_init->get(1).asDouble();
            shoulderRight_init[2] = bShoulderRight_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for shoulder right";

        if(Bottle *bHipLeft_init = bGeneral.find("hip_left_init_pose").asList())
        {
            hipLeft_init[0] = bHipLeft_init->get(0).asDouble();
            hipLeft_init[1] = bHipLeft_init->get(1).asDouble();
            hipLeft_init[2] = bHipLeft_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for hip left";

        if(Bottle *bhipRight_init = bGeneral.find("hip_right_init_pose").asList())
        {
            hipRight_init[0] = bhipRight_init->get(0).asDouble();
            hipRight_init[1] = bhipRight_init->get(1).asDouble();
            hipRight_init[2] = bhipRight_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for hip right";

        if(Bottle *bKneeLeft_init = bGeneral.find("knee_left_init_pose").asList())
        {
            kneeLeft_init[0] = bKneeLeft_init->get(0).asDouble();
            kneeLeft_init[1] = bKneeLeft_init->get(1).asDouble();
            kneeLeft_init[2] = bKneeLeft_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for knee left";

        if(Bottle *bKneeRight_init = bGeneral.find("knee_right_init_pose").asList())
        {
            kneeRight_init[0] = bKneeRight_init->get(0).asDouble();
            kneeRight_init[1] = bKneeRight_init->get(1).asDouble();
            kneeRight_init[2] = bKneeRight_init->get(2).asDouble();
        }
        else
            yError() << "Could not load initial configuration for knee right";


        processor = new Processor(elbowLeft_init, elbowRight_init, handLeft_init, handRight_init,
                                  head_init, shoulderCenter_init, shoulderLeft_init, shoulderRight_init,
                                  hipLeft_init, hipRight_init, kneeLeft_init, kneeRight_init);
    }

}

bool Manager::load()
{
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext(this->rf->getContext().c_str());
    rf.setDefaultConfigFile(this->rf->find("configuration-file").asString().c_str());
    rf.configure(0, NULL);

    Bottle &bGeneral = rf.findGroup("GENERAL");

    if(!bGeneral.isNull())
    {
        if(Bottle *motion_tag = bGeneral.find("motion_tag").asList())
        {
            if(Bottle *n_motion_tag = bGeneral.find("number_motion").asList())
            {
                for(int i=0; i<motion_tag->size(); i++)
                {
                    string curr_tag = motion_tag->get(i).asString();
                    int motion_number = n_motion_tag->get(i).asInt();

                    for(int j=0; j<motion_number; j++)
                    {
                        Bottle &bMotion = rf.findGroup(curr_tag+"_"+to_string(j));
                        if(!bMotion.isNull())
                        {
                            if(Bottle *bJoint = bMotion.find("tag_joint").asList())
                            {
                                Metric *cMetric = NULL;
                                if(curr_tag == "ROM")
                                {
                                    string tag_joint = bJoint->get(0).asString();
                                    int id_joint = bJoint->get(1).asInt();
                                    int n_motion = motion_number;
                                    double min = bMotion.find("min").asDouble();
                                    double max = bMotion.find("max").asDouble();

                                    Bottle *elbowLC = bMotion.find("elbow_left_configuration").asList();
                                    if(elbowLC)
                                    {
                                        string eLC = elbowLC->get(0).asString();
                                        if(eLC == "mobile")
                                        {
                                            elbowLeft[3] = 1;
                                            elbowLeft[4] = elbowLC->get(1).asDouble();
                                            elbowLeft[5] = elbowLC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            elbowLeft[3] = 0;
                                            elbowLeft[4] = 0;
                                            elbowLeft[5] = 0;
                                        }

                                    }
                                    else
                                        yError() << "Could not load elbow left configuration";

                                    Bottle *elbowRC = bMotion.find("elbow_right_configuration").asList();
                                    if(elbowRC)
                                    {
                                        string eRC = elbowRC->get(0).asString();
                                        if(eRC == "mobile")
                                        {
                                            elbowRight[3] = 1;
                                            elbowRight[4] = elbowRC->get(1).asDouble();
                                            elbowRight[5] = elbowRC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            elbowRight[3] = 0;
                                            elbowRight[4] = 0;
                                            elbowRight[5] = 0;
                                        }
                                    }
                                    else
                                        yError() << "Could not load elbow right configuration";

                                    Bottle *handLC = bMotion.find("hand_left_configuration").asList();
                                    if(handLC)
                                    {
                                        string hnLC = handLC->get(0).asString();
                                        if(hnLC == "mobile")
                                        {
                                            handLeft[3] = 1;
                                            handLeft[4] = handLC->get(1).asDouble();
                                            handLeft[5] = handLC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            handLeft[3] = 0;
                                            handLeft[4] = 0;
                                            handLeft[5] = 0;
                                        }

                                    }
                                    else
                                        yError() << "Could not load hand left configuration";

                                    Bottle *handRC = bMotion.find("hand_right_configuration").asList();
                                    if(handRC)
                                    {
                                        string hnRC = handLC->get(0).asString();
                                        if(hnRC == "mobile")
                                        {
                                            handRight[3] = 1;
                                            handRight[4] = handRC->get(1).asDouble();
                                            handRight[5] = handRC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            handRight[3] = 0;
                                            handRight[4] = 0;
                                            handRight[5] = 0;
                                        }

                                    }
                                    else
                                        yError() << "Could not load hand right configuration";

                                    Bottle *headC = bMotion.find("head_configuration").asList();
                                    if(headC)
                                    {
                                        string hC = headC->get(0).asString();
                                        if(hC == "mobile")
                                        {
                                            head[3] = 1;
                                            head[4] = headC->get(1).asDouble();
                                            head[5] = headC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            head[3] = 0;
                                            head[4] = 0;
                                            head[5] = 0;
                                        }

                                    }
                                    else
                                        yError() << "Could not load head configuration";

                                    Bottle *shoulderCC = bMotion.find("shoulder_center_configuration").asList();
                                    if(shoulderCC)
                                    {
                                        string sCC = shoulderCC->get(0).asString();
                                        if(sCC == "mobile")
                                        {
                                            shoulderCenter[3] = 1;
                                            shoulderCenter[4] = shoulderCC->get(1).asDouble();
                                            shoulderCenter[5] = shoulderCC->get(2).asDouble();

                                        }
                                        else
                                        {
                                            shoulderCenter[3] = 0;
                                            shoulderCenter[4] = 0;
                                            shoulderCenter[5] = 0;
                                        }

                                    }
                                    else
                                        yError() << "Could not load shoulder center configuration";

                                    Bottle *shoulderLC = bMotion.find("shoulder_left_configuration").asList();
                                    if(shoulderLC)
                                    {
                                        string sLC = shoulderLC->get(0).asString();
                                        if(sLC == "mobile")
                                        {
                                            shoulderLeft[3] = 1;
                                            shoulderLeft[4] = shoulderLC->get(1).asDouble();
                                            shoulderLeft[5] = shoulderLC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            shoulderLeft[3] = 0;
                                            shoulderLeft[4] = 0;
                                            shoulderLeft[5] = 0;
                                        }

                                    }
                                    else
                                        yError() << "Could not load shoulder left configuration";

                                    Bottle *shoulderRC = bMotion.find("shoulder_right_configuration").asList();
                                    if(shoulderRC)
                                    {
                                        string sRC = shoulderRC->get(0).asString();
                                        if(sRC == "mobile")
                                        {
                                            shoulderRight[3] = 1;
                                            shoulderRight[4] = shoulderRC->get(1).asDouble();
                                            shoulderRight[5] = shoulderRC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            shoulderRight[3] = 0;
                                            shoulderRight[4] = 0;
                                            shoulderRight[5] = 0;
                                        }
                                    }
                                    else
                                        yError() << "Could not load shoulder right configuration";

                                    Bottle *hipLC = bMotion.find("hip_left_configuration").asList();                                        if(shoulderCC)
                                        if(hipLC)
                                        {
                                            string hLC = hipLC->get(0).asString();
                                            if(hLC == "mobile")
                                            {
                                                hipLeft[3] = 1;
                                                hipLeft[4] = hipLC->get(1).asDouble();
                                                hipLeft[5] = hipLC->get(2).asDouble();
                                            }
                                            else
                                            {
                                                hipLeft[3] = 0;
                                                hipLeft[4] = 0;
                                                hipLeft[5] = 0;
                                            }
                                        }
                                        else
                                            yError() << "Could not load hip left configuration";

                                    Bottle *hipRC = bMotion.find("hip_right_configuration").asList();                                        if(shoulderRC)
                                        if(hipRC)
                                        {
                                            string hRC = hipRC->get(0).asString();
                                            if(hRC == "mobile")
                                            {
                                                hipRight[3] = 1;
                                                hipRight[4] = hipRC->get(1).asDouble();
                                                hipRight[5] = hipRC->get(2).asDouble();
                                            }
                                            else
                                            {
                                                hipRight[3] = 0;
                                                hipRight[4] = 0;
                                                hipRight[5] = 0;
                                            }
                                        }
                                        else
                                            yError() << "Could not load hip right configuration";

                                    Bottle *kneeLC = bMotion.find("knee_left_configuration").asList();
                                    if(kneeLC)
                                    {
                                        string kLC = kneeLC->get(0).asString();
                                        if(kLC == "mobile")
                                        {
                                            kneeLeft[3] = 1;
                                            kneeLeft[4] = kneeLC->get(1).asDouble();
                                            kneeLeft[5] = kneeLC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            kneeLeft[3] = 0;
                                            kneeLeft[4] = 0;
                                            kneeLeft[5] = 0;
                                        }
                                    }
                                    else
                                        yError() << "Could not load knee left configuration";

                                    Bottle *kneeRC = bMotion.find("knee_right_configuration").asList();
                                    if(kneeRC)
                                    {
                                        string kRC = kneeRC->get(0).asString();
                                        if(kRC == "mobile")
                                        {
                                            kneeRight[3] = 1;
                                            kneeRight[4] = kneeRC->get(1).asDouble();
                                            kneeRight[5] = kneeRC->get(2).asDouble();
                                        }
                                        else
                                        {
                                            kneeRight[3] = 0;
                                            kneeRight[3] = 0;
                                            kneeRight[3] = 0;
                                        }
                                    }
                                    else
                                        yError() << "Could not load knee right configuration";

                                    rom = new Rom(tag_joint, id_joint, n_motion, min, max,
                                                  elbowLeft, elbowRight, handLeft, handRight,
                                                  head, shoulderCenter, shoulderLeft, shoulderRight,
                                                  hipLeft, hipRight, kneeLeft, kneeRight);
                                    cMetric = rom;
                                }

                                motion_list.insert(pair<string, Metric*>(curr_tag+"_"+to_string(j), cMetric));
//                                motion_list[curr_tag+"_"+to_string(j)]->print();

                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        yError() << "Error in loading parameters. Stopping module!";
        return false;
    }

    return true;
}

/********************************************************/
void Manager::getKeyframes()
{
    //ask for the property id
    Bottle cmd, reply;
    cmd.addVocab(Vocab::encode("ask"));
    Bottle &content = cmd.addList().addList();
    content.addString("body");

//    yInfo() << "Query opc: " << cmd.toString();
    opcPort.write(cmd, reply);
//    yInfo() << "Reply from opc:" << reply.toString();

    if(reply.size() > 1)
    {
        if(reply.get(0).asVocab() == Vocab::encode("ack"))
        {
            if(Bottle *idField = reply.get(1).asList())
            {
                if(Bottle *idValues = idField->get(1).asList())
                {
                    //get the last id
                    int id = idValues->get(idValues->size()-1).asInt();
                    //                        yInfo() << id;

                    //given the id, get the value of the property
                    cmd.clear();
                    cmd.addVocab(Vocab::encode("get"));
                    Bottle &content = cmd.addList().addList();
                    content.addString("id");
                    content.addInt(id);
                    Bottle replyProp;

                    //                        yInfo() << "Command sent to the port: " << cmd.toString();
                    opcPort.write(cmd, replyProp);
                    //                        yInfo() << "Reply from opc:" << replyProp.toString();

                    if(replyProp.get(0).asVocab() == Vocab::encode("ack"))
                    {
                        if(Bottle *propField = replyProp.get(1).asList())
                        {
                            if(Bottle *propSubField = propField->find("body").asList())
                            {
                                if(Bottle *elbowLeftB = propSubField->find("elbowLeft").asList())
                                {
                                    elbowLeft[0] = elbowLeftB->get(0).asDouble();
                                    elbowLeft[1] = elbowLeftB->get(1).asDouble();
                                    elbowLeft[2] = elbowLeftB->get(2).asDouble();
//                                    yInfo() << elbowLeft[0] << elbowLeft[1] << elbowLeft[2];
                                }
                                else
                                    yError() << "Could not read elbow left";

                                if(Bottle *elbowRightB = propSubField->find("elbowRight").asList())
                                {
                                    elbowRight[0] = elbowRightB->get(0).asDouble();
                                    elbowRight[1] = elbowRightB->get(1).asDouble();
                                    elbowRight[2] = elbowRightB->get(2).asDouble();
//                                    yInfo() << elbowRight[0] << elbowRight[1] << elbowRight[2];
                                }
                                else
                                    yError() << "Could not read elbow right";

                                if(Bottle *handLeftB = propSubField->find("handLeft").asList())
                                {
                                    handLeft[0] = handLeftB->get(0).asDouble();
                                    handLeft[1] = handLeftB->get(1).asDouble();
                                    handLeft[2] = handLeftB->get(2).asDouble();
//                                    yInfo() << handLeft[0] << handLeft[1] << handLeft[2];
                                }
                                else
                                    yError() << "Could not read hand left";

                                if(Bottle *handRightB = propSubField->find("handRight").asList())
                                {
                                    handRight[0] = handRightB->get(0).asDouble();
                                    handRight[1] = handRightB->get(1).asDouble();
                                    handRight[2] = handRightB->get(2).asDouble();
//                                    yInfo() << handRight[0] << handRight[1] << handRight[2];
                                }
                                else
                                    yError() << "Could not read hand right";

                                if(Bottle *headB = propSubField->find("head").asList())
                                {
                                    head[0] = headB->get(0).asDouble();
                                    head[1] = headB->get(1).asDouble();
                                    head[2] = headB->get(2).asDouble();
//                                    yInfo() << head[0] << head[1] << head[2];
                                }
                                else
                                    yError() << "Could not read hand left";

                                if(Bottle *shoulderCenterB = propSubField->find("shoulderCenter").asList())
                                {
                                    shoulderCenter[0] = shoulderCenterB->get(0).asDouble();
                                    shoulderCenter[1] = shoulderCenterB->get(1).asDouble();
                                    shoulderCenter[2] = shoulderCenterB->get(2).asDouble();
//                                    yInfo() << shoulderCenter[0] << shoulderCenter[1] << shoulderCenter[2];
                                }
                                else
                                    yError() << "Could not read shoulder center";

                                if(Bottle *shoulderLeftB = propSubField->find("shoulderLeft").asList())
                                {
                                    shoulderLeft[0] = shoulderLeftB->get(0).asDouble();
                                    shoulderLeft[1] = shoulderLeftB->get(1).asDouble();
                                    shoulderLeft[2] = shoulderLeftB->get(2).asDouble();
//                                    yInfo() << shoulderLeft[0] << shoulderLeft[1] << shoulderLeft[2];
                                }
                                else
                                    yError() << "Could not read shoulder left";

                                if(Bottle *shoulderRightB = propSubField->find("shoulderRight").asList())
                                {
                                    shoulderRight[0] = shoulderRightB->get(0).asDouble();
                                    shoulderRight[1] = shoulderRightB->get(1).asDouble();
                                    shoulderRight[2] = shoulderRightB->get(2).asDouble();
//                                    yInfo() << shoulderRight[0] << shoulderRight[1] << shoulderRight[2];
                                }
                                else
                                    yError() << "Could not read shoulder right";

                                if(Bottle *hipLeftB = propSubField->find("hipLeft").asList())
                                {
                                    hipLeft[0] = hipLeftB->get(0).asDouble();
                                    hipLeft[1] = hipLeftB->get(1).asDouble();
                                    hipLeft[2] = hipLeftB->get(2).asDouble();
//                                    yInfo() << hipLeft[0] << hipLeft[1] << hipLeft[2];
                                }
                                else
                                    yError() << "Could not read hip left";

                                if(Bottle *hipRightB = propSubField->find("hipRight").asList())
                                {
                                    hipRight[0] = hipRightB->get(0).asDouble();
                                    hipRight[1] = hipRightB->get(1).asDouble();
                                    hipRight[2] = hipRightB->get(2).asDouble();
//                                    yInfo() << hipRight[0] << hipRight[1] << hipRight[2];
                                }
                                else
                                    yError() << "Could not read hip right";

                                if(Bottle *kneeLeftB = propSubField->find("kneeLeft").asList())
                                {
                                    kneeLeft[0] = kneeLeftB->get(0).asDouble();
                                    kneeLeft[1] = kneeLeftB->get(1).asDouble();
                                    kneeLeft[2] = kneeLeftB->get(2).asDouble();
//                                    yInfo() << kneeLeft[0] << kneeLeft[1] << kneeLeft[2];
                                }
                                else
                                    yError() << "Could not read knee left";

                                if(Bottle *kneeRightB = propSubField->find("kneeRight").asList())
                                {
                                    kneeRight[0] = kneeRightB->get(0).asDouble();
                                    kneeRight[1] = kneeRightB->get(1).asDouble();
                                    kneeRight[2] = kneeRightB->get(2).asDouble();
//                                    yInfo() << kneeRight[0] << kneeRight[1] << kneeRight[2];
                                }
                                else
                                    yError() << "Could not read knee right";

                            }
                        }
                    }
                }
            }
        }
    }
}

/********************************************************/
void Manager::mapKeyframesToStandard()
{

}

/********************************************************/
bool Manager::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/********************************************************/
bool Manager::configure(ResourceFinder &rf)
{
    this->rf = &rf;
    string moduleName = rf.check("name", Value("motionAnalyzer")).asString();
    setName(moduleName.c_str());

    string robot = rf.check("robot", Value("icub")).asString();

    opcPort.open(("/" + getName() + "/opc").c_str());
    scopePort.open(("/" + getName() + "/scope").c_str());
    rpcPort.open(("/" + getName() + "/cmd").c_str());
    attach(rpcPort);

    init();
    loadInitialConf();
    if(!load())
        return false;

    return true;
}

/********************************************************/
bool Manager::close()
{
    delete rom;
    delete processor;

    opcPort.close();
    scopePort.close();
    rpcPort.close();

    return true;
}

/********************************************************/
double Manager::getPeriod()
{
    return 1.0;
}

/********************************************************/
bool Manager::updateModule()
{
    //if we query the database
    if(opcPort.getOutputCount() > 0)
    {
        getKeyframes();
//        mapKeyframesToStandard();
        rom->update(elbowLeft, elbowRight, handLeft, handRight,
                    head, shoulderCenter, shoulderLeft, shoulderRight,
                    hipLeft, hipRight, kneeLeft, kneeRight);
//        rom->print();

        Rom_Processor rom_processor(rom);
//        rom_processor.checkDeviationFromIntialPose();

        //                    //write it on the output
        //                    Bottle &scopebottleout = scopePort.prepare();
        //                    scopebottleout.clear();
        //                    scopebottleout.addDouble(rom);
        //                    scopePort.write();
    }

    return true;
}

