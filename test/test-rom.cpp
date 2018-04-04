/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-viewer.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <cmath>
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

class TestRom : public RFModule
{
    SkeletonWaist skeleton;
    double radius,phase,t0;
    BufferedPort<Bottle> port;

    bool configure(ResourceFinder &rf) override
    {
        skeleton.setTag("test-rom");

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
            Vector p(3,0.0); p[0]=0.1; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::shoulder_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.2; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::elbow_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.3; p[1]=0.0; p[2]=0.0;
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
        {
            Vector p(3,0.0); p[0]=-0.1; p[1]=-0.2; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::hand_right,p));
        }
        skeleton.update(unordered);

        Vector d=skeleton[KeyPointTag::head]->getPoint()-
                 skeleton[KeyPointTag::shoulder_center]->getPoint();
        radius=norm(d);
        phase=atan2(d[1],d[0]);

        port.open("/test-rom");
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

        Vector c=skeleton[KeyPointTag::shoulder_right]->getPoint();
        double theta = 2*M_PI*0.1*t;

        Vector p(3,0.0);
        p[0]=-radius*cos(theta + phase);
        if(p[0] > 0.0)
            p[0]=-p[0];
        p[0]=c[0]+p[0];

        p[1]=c[1]-radius*fabs(sin(theta + phase));

//        p[0]+=radius*cos(2*M_PI*0.1*(t-t0)); //+phase);
//        p[1]+=radius*sin(2*M_PI*0.1*(t-t0)); //+phase);

//        p[1]+=radius*sin(2*M_PI*0.1*t); //+phase);
//        p[2]+=radius*cos(2*M_PI*0.1*t); //+phase);

        vector<pair<string,Vector>> unordered=skeleton.get_unordered();
        unordered.push_back(make_pair(KeyPointTag::elbow_right,p));

        p[0]=-2*radius*cos(theta + phase);
        if(p[0] > 0.0)
            p[0]=-p[0];
        p[0]=c[0]+p[0];

        p[1]=c[1]-2*radius*fabs(sin(theta + phase));
        unordered.push_back(make_pair(KeyPointTag::hand_right,p));

        skeleton.update(unordered);

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

    TestRom testrom;
    ResourceFinder rf;
    return testrom.runModule(rf);
}
