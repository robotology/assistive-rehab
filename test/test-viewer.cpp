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
    SkeletonStd skeleton1,skeleton2;
    double radius,phase,t0;
    BufferedPort<Bottle> port;

    bool configure(ResourceFinder &rf) override
    {
        skeleton1.setTag("test-1");
        skeleton2.setTag("test-2");

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
            Vector p(3,0.0); p[0]=-0.2; p[1]=0.0; p[2]=0.0;
            unordered.push_back(make_pair(KeyPointTag::elbow_right,p));
        }
        {
            Vector p(3,0.0); p[0]=-0.3; p[1]=0.0; p[2]=0.0;
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
        rot.push_back(2.0*M_PI*0.1*t);
        Matrix T=axis2dcm(rot);
        T.setSubcol(0.4*skeleton1.getCoronal(),0,3);
        skeleton2.setTransformation(T);
        skeleton2.update(skeleton1.get_ordered());

        Property prop1=skeleton1.toProperty();
        Property prop2=skeleton2.toProperty();
        prop2.put("opacity",0.25+0.75*(1.0-cos(theta))/2.0);

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
