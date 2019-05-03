/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-viewer.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <cmath>
#include <random>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class TestViewer : public RFModule
{
    SkeletonStd skeleton1;
    SkeletonStd skeleton2;
    double radius,phase,t0;
    BufferedPort<Bottle> port;

    random_device rd;
    mt19937 mt;
    uniform_real_distribution<double> dist;

    bool configure(ResourceFinder &rf) override
    {
        skeleton1.setTag("test-1");
        skeleton2.setTag("test-2");

        vector<pair<string, Vector>> unordered;
        {
            Vector p(3,0.0); p[0]=0.0; p[1]=0.0; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_center,p));
        }
        {
            Vector p(3,0.0); p[0]=0.0; p[1]=-0.1; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::head,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=0.0; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.2; p[1]=0.0; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::elbow_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.3; p[1]=0.0; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::hand_left,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=0.0; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.2; p[1]=0.0; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::elbow_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.3; p[1]=0.0; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::hand_right,p));
        }
        {
            Vector p(3,0.0); p[0]=0.0; p[1]=0.1; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::hip_center,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=0.1; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::hip_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=0.2; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::knee_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=0.3; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::ankle_left,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=0.1; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::hip_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=0.2; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::knee_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=0.3; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::ankle_right,p));
        }
        skeleton1.update(unordered);

        Vector d=skeleton1[KeyPointTag::head]->getPoint()-
                 skeleton1[KeyPointTag::shoulder_center]->getPoint();
        radius=norm(d);
        phase=atan2(d[1],d[0]);

        port.open("/test-viewer");
        t0=Time::now();

        return true;
    }

    double getPeriod() override
    {
        return 0.01;
    }

    bool updateModule() override
    {
        double t=Time::now()-t0;
        double theta=2.0*M_PI*0.1*t+phase;

        Vector p=skeleton1[KeyPointTag::shoulder_center]->getPoint();
        p[0]+=radius*cos(theta);
        p[1]+=radius*sin(theta);

        vector<pair<string,Vector>> unordered=skeleton1.get_unordered();
        unordered.erase(unordered.end()-2,unordered.end());
        unordered.push_back(make_pair(KeyPointTag::head,p));
        skeleton1.update(unordered);

        Vector rot=skeleton1.getTransverse();
        rot.push_back(theta);
        Matrix T=axis2dcm(rot);
        skeleton2.setTransformation(T);
        skeleton2.update(skeleton1.get_ordered());

        Property prop1=skeleton1.toProperty();
        Property prop2=skeleton2.toProperty();
        
        Bottle color;
        color.addList().read(Vector(3,dist(mt)));

        prop2.put("opacity",0.25+0.75*(1.0-cos(theta))/2.0);
        prop2.put("color",color.get(0));

        Bottle &msg=port.prepare();
        msg.clear();
        msg.addList().read(prop1);
        msg.addList().read(prop2);
        port.write();

        return true;
    }

    bool close() override
    {
        port.close();
        return true;
    }

public:
    TestViewer() : mt(rd()), dist(0.0,1.0) { }
};

int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    TestViewer testviewer;
    ResourceFinder rf;
    return testviewer.runModule(rf);
}

