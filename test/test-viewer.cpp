/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-skeleton1.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <cmath>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class TestViewer : public RFModule
{
    SkeletonStd skeleton;
    double radius,phase,t0;
    BufferedPort<Property> port;

    bool configure(ResourceFinder &rf) override
    {
        skeleton.setTag("test");
        vector<pair<string, Vector>> unordered;
        {
            Vector p(3,0.0); p[0]=0.0; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_center,p));
        }
        {
            Vector p(3,0.0); p[0]=0.0; p[1]=0.1; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::head,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_left,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::elbow_left,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hand_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_right,p));
        }
        {
            Vector p(3,0.0); p[0]=0.2; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::elbow_right,p));
        }
        {
            Vector p(3,0.0); p[0]=0.3; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hand_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.1; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hip_left,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.2; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::knee_left,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.3; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::ankle_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=-0.1; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hip_right,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=-0.2; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::knee_right,p));
        }
        {
            Vector p(3,0.0); p[0]=0.1; p[1]=-0.3; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::ankle_right,p));
        }
        skeleton.update(unordered);

        Vector d=skeleton[KeyPointTag::head]->getPoint()-
                 skeleton[KeyPointTag::shoulder_center]->getPoint();
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

        Vector p=skeleton[KeyPointTag::shoulder_center]->getPoint();
        p[0]+=radius*cos(2.0*M_PI*0.1*t+phase);
        p[1]+=radius*sin(2.0*M_PI*0.1*t+phase);

        vector<pair<string,Vector>> unordered;
        unordered.push_back(make_pair(KeyPointTag::head,p));
        skeleton.update(unordered);

        port.prepare()=skeleton.toProperty();
        port.write();

        return true;
    }

    bool close() override
    {
        port.close();
        return true;
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

    TestViewer testviewer;
    ResourceFinder rf;
    return testviewer.runModule(rf);
}
