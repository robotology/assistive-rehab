/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-rom.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <cmath>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include "AssistiveRehab/skeleton.h"

//#include "src/rom-test_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class TestRom : public RFModule
{
    SkeletonWaist skeleton;
    double radius,phase,t0;
    BufferedPort<Bottle> port;
    RpcServer rpcPort;

    string joint, motiontype;
    bool move;

    bool configure(ResourceFinder &rf) override
    {
        skeleton.setTag("test-rom");

        vector<pair<string, Vector>> unordered;
        setPose(unordered);
        skeleton.update(unordered);

        Vector d=skeleton[KeyPointTag::head]->getPoint()-
                 skeleton[KeyPointTag::shoulder_center]->getPoint();
        radius=norm(d);
        phase=atan2(d[1],d[0]);

        port.open("/test-rom");

        rpcPort.open("/test-rom/rpc");
        attach(rpcPort);

        t0=Time::now();

        return true;
    }

    double getPeriod() override
    {
        return 0.01;
    }

    bool updateModule() override
    {
        if(isMoving())
        {
            vector< Vector > p;
            vector<const KeyPoint*> children;

            generateTarget(p, children);

            vector<pair<string,Vector>> unordered=skeleton.get_unordered();
            for(int i=0; i<unordered.size(); i++)
            {
                for(int j=0; j<children.size(); j++)
                {
//                    cout << children[j]->getTag().c_str() << " " << children[j]->getPoint()[0] << " " <<
//                            children[j]->getPoint()[1] << " " << children[j]->getPoint()[2] << endl;
                    if(unordered[i].first == children[j]->getTag().c_str())
                        unordered[i] = make_pair(children[j]->getTag(),p[j]);
                }
            }

            skeleton.update(unordered);
        }

        Property prop=skeleton.toProperty();
        Bottle &msg=port.prepare();
        msg.clear();
        msg.addList().read(prop);
        port.write();

        return true;
    }

    bool close() override
    {
        port.close();
        rpcPort.close();
        return true;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        if(command.get(0).asString() == "set")
        {
            stopMotion();
            setJoint(command.get(1).asString());
            setMotionType(command.get(2).asString());
            reply.addString("Setting initial pose for " + command.get(1).asString() + " and " + command.get(2).asString());
            setInitialPose();
        }
        if(command.get(0).asString() == "moveJoint")
        {
            setJoint(command.get(1).asString());
            Bottle* pos=command.get(2).asList();
            moveJoint(joint, pos);
            reply.addString("Move joint " + command.get(1).asString());

        }
        else if(command.get(0).asString() == "move")
        {
            reply.addString("Starting motion");
            startMotion();
        }
        else if(command.get(0).asString() == "stop")
        {
            reply.addString("Stopping motion");
            stopMotion();
        }

        return true;
    }

    void setJoint(const string &joint_)
    {
        joint = joint_;
    }

    void setMotionType(const string &motiontype_)
    {
        motiontype = motiontype_;
    }

    void startMotion()
    {
        move = true;
    }

    void stopMotion()
    {
        move = false;
    }

    bool isMoving()
    {
        return move;
    }

    void moveJoint(const string& joint_test_, Bottle* pos)
    {
        vector<pair<string,Vector>> unordered=skeleton.get_unordered();
        Vector p(3,0.0);
        p[0]=pos->get(0).asDouble();
        p[1]=pos->get(1).asDouble();
        p[2]=pos->get(2).asDouble();

        for(int i=0; i<unordered.size(); i++)
        {
            if(unordered[i].first == joint_test_.c_str())
                unordered[i] = make_pair(joint_test_.c_str(),p);
        }
        skeleton.update(unordered);
    }

    void generateTarget(vector< Vector > &p, vector<const KeyPoint*> &children)
    {
        double t=Time::now()-t0;
        double theta = 2*M_PI*0.1*t;

        Vector c(skeleton[joint]->getPoint());
        int numchild = skeleton[joint]->getNumChild();
        int totnumchild=0;

        totnumchild+=numchild;
        for(int j=0; j<numchild; j++)
        {
            const KeyPoint *keypoint=skeleton[joint]->getChild(j);
            children.push_back(keypoint);

            int numchild2=skeleton[keypoint->getTag()]->getNumChild();
            if(numchild2)
            {
                const KeyPoint* keypoint2=skeleton[keypoint->getTag()]->getChild(0);
                children.push_back(keypoint2);
                totnumchild+=numchild2;
            }
        }

        p.resize(totnumchild);
        if(motiontype == "internalRotation")
        {
            for(int j=0; j<totnumchild; j++)
            {
                p[j].resize(3);
                p[j][0]=c[0];
                p[j][1]=(j+1)*radius*(cos(theta+phase));
                if(p[j][1] > 0.0)
                    p[j][1]=-p[j][1];
                p[j][1]+=c[1];

                p[j][2]=c[2]+(j+1)*radius*fabs(sin(theta+phase));
            }
        }
        else if(motiontype == "externalRotation")
        {
            for(int j=0; j<totnumchild; j++)
            {
                p[j].resize(3);
                p[j][0]=c[0];
                p[j][1]=c[1]+(j+1)*radius*fabs(cos(theta+phase));
                p[j][2]=c[2]+(j+1)*radius*fabs(sin(theta+phase));
            }
        }
        else if(motiontype == "abduction")
        {
            if(joint.find("Right") != string::npos)
            {
                for(int j=0; j<totnumchild; j++)
                {
                    p[j].resize(3);
                    p[j][0]=-(j+1)*radius*cos(theta+phase);
                    if(p[j][0] > 0.0)
                        p[j][0]=-p[j][0];
                    p[j][0]+=c[0];

                    p[j][1]=c[1]-(j+1)*radius*fabs(sin(theta+phase));
                    p[j][2]=c[2];
                }
            }
            else
            {
                for(int j=0; j<totnumchild; j++)
                {
                    p[j].resize(3);
                    p[j][0]=c[0]+(j+1)*radius*fabs(sin(theta+phase));

                    p[j][1]=(j+1)*radius*cos(theta+phase);
                    if(p[j][1] > 0.0)
                        p[j][1]=-p[j][1];
                    p[j][1]+=c[1];

                    p[j][2]=c[2];
                }
            }
        }
        else if(motiontype == "flexion")
        {
            for(int j=0; j<totnumchild; j++)
            {
                p[j].resize(3);
                p[j][0]=c[0];
                p[j][1]=c[1]+(j+1)*radius*fabs(sin(theta+phase));
                p[j][2]=c[2]+(j+1)*radius*fabs(cos(theta+phase));
            }
        }
        else if(motiontype == "extension")
        {
            for(int j=0; j<totnumchild; j++)
            {
                p[j].resize(3);
                p[j][0]=c[0];
                p[j][1]=(j+1)*radius*sin(theta+phase);
                if(p[j][1] > 0.0)
                    p[j][1]=-p[j][1];
                p[j][1]+=c[1];

                p[j][2]=c[2]+(j+1)*radius*fabs(cos(theta+phase));
            }
        }
    }

    void setInitialPose()
    {
        vector<pair<string,Vector>> unordered=skeleton.get_unordered();
        if(motiontype == "internalRotation" || motiontype == "externalRotation")
        {
            if(joint.find("Right") != string::npos)
            {
                vector<string> joint_v;
                joint_v.push_back(joint);
                joint_v.push_back(skeleton[joint]->getChild(0)->getTag());

                vector<Vector> p(joint_v.size(),Vector(3));
                p[0][0]=-0.2;
                p[0][1]=0.0;
                p[0][2]=0.0;

                p[1][0]=-0.2;
                p[1][1]=0.0;
                p[1][2]=0.1;

                setPose(unordered, joint_v, p);
            }
            else
            {
                vector<string> joint_v;
                joint_v.push_back(joint);
                joint_v.push_back(skeleton[joint]->getChild(0)->getTag());

                vector<Vector> p(joint_v.size(),Vector(3));
                p[0][0]=0.2;
                p[0][1]=0.0;
                p[0][2]=0.0;

                p[1][0]=0.2;
                p[1][1]=0.0;
                p[1][2]=0.1;

                setPose(unordered, joint_v, p);
            }
        }        else
        {
           setPose(unordered);
        }

        skeleton.update(unordered);

    }

    void setPose(vector<pair<string, Vector>>& unordered)
    {
        {
            Vector p(3,0.0); p[0]=0.0; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_center,p));
        }
        {
            Vector p(3,0.0); p[0]=0.0; p[1]=0.1; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::head,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=-0.1; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::elbow_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=-0.2; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hand_left,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.1; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::elbow_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.2; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hand_right,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=-0.1; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hip_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=-0.2; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::knee_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=-0.3; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::ankle_left,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.1; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hip_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.2; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::knee_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.3; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::ankle_right,p));
        }
    }

    void setPose(vector<pair<string, Vector>>& unordered, const vector<string>& joint_v, const vector<Vector>& p)
    {
        setPose(unordered);
        for(int i=0; i<unordered.size(); i++)
        {
            for(int j=0; j<joint_v.size(); j++)
            {
                if(unordered[i].first == joint_v[j].c_str())
                {
                    unordered[i] = make_pair(joint_v[j].c_str(),p[j]);
                }
            }
        }
    }

    void helper_gettree(const KeyPoint* k, vector<KeyPoint> &helperpoints)
    {

//        int numchild = k->getNumChild();
//        int numchild2;
//        for(int j=0; j<numchild; j++)
//        {
//            const KeyPoint* keypoint=skeleton[k->getTag()]->getChild(j);
//            helperpoints.push_back(*keypoint);

//            numchild2=skeleton[keypoint->getTag()]->getNumChild();
//            if(numchild2)
//            {
//                for(int k=0; k<numchild2; k++)
//                {
//                    const KeyPoint* keypoint2=skeleton[keypoint->getTag()]->getChild(k);
//                    helperpoints.push_back(*keypoint2);
//                }
//            }
//            else
//                break;
//        }
//        numchildren=numchild+numchild2;

    }

};

int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    TestRom testrom;
    ResourceFinder rf;
    return testrom.runModule(rf);
}
