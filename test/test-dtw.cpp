/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-dtw.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <fstream>
#include <cstdlib>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include "../modules/alignmentManager/include/Dtw.h"
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class TestDtw : public RFModule
{
private:
    double amplitude,phase;
    int win;
    bool align;
    SkeletonWaist skeleton1,skeleton2,alignedSkel;
    BufferedPort<Bottle> port_out,scopePort;
    double radius,phi,t0;
    vector<Vector> skeleton_template,skeleton_candidate,aligned_skeleton;
    vector<double> t1series,t2series,tAligned;
    int nsamples;
//    ofstream file_template,file_skeleton;
    Dtw dtw;

public:

    bool configure(ResourceFinder &rf) override
    {
//        file_template.open("template.txt");
//        file_skeleton.open("skeleton.txt");

        amplitude = rf.check("amplitude",Value(1.0)).asDouble();
        phase = rf.check("phase",Value(M_PI/2.0)).asDouble();
        win = rf.check("win",Value(5)).asDouble();
        align = rf.check("align",Value(true)).asBool();
        nsamples = rf.check("nsamples",Value(200)).asDouble();

        dtw.initialize(win,nsamples,nsamples,45);

        skeleton1.setTag("template");
        skeleton2.setTag("skeleton");
        alignedSkel.setTag("aligned");
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
            Vector p(3,0.0); p[0]=0.1; p[1]=0.1; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::hip_left,p));
        }
        {
            Vector p(3,0.0); p[0]=0.0; p[1]=0.1; p[2]=1.0;
            unordered.push_back(make_pair(KeyPointTag::hip_center,p));
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
        skeleton2.update(unordered);

        Vector d=skeleton1[KeyPointTag::head]->getPoint()-
                 skeleton1[KeyPointTag::shoulder_center]->getPoint();
        radius=norm(d);
        phi=atan2(d[1],d[0]);

        port_out.open("/test-dtw");
        scopePort.open("/test-dtw/scope");
        t0=Time::now();

        return true;
    }

    void createVec(const SkeletonWaist& skeleton, Vector& temp)
    {
        temp[0] = skeleton[KeyPointTag::shoulder_center]->getPoint()[0];
        temp[1] = skeleton[KeyPointTag::shoulder_center]->getPoint()[1];
        temp[2] = skeleton[KeyPointTag::shoulder_center]->getPoint()[2];

        temp[3] = skeleton[KeyPointTag::head]->getPoint()[0];
        temp[4] = skeleton[KeyPointTag::head]->getPoint()[1];
        temp[5] = skeleton[KeyPointTag::head]->getPoint()[2];

        temp[6] = skeleton[KeyPointTag::shoulder_left]->getPoint()[0];
        temp[7] = skeleton[KeyPointTag::shoulder_left]->getPoint()[1];
        temp[8] = skeleton[KeyPointTag::shoulder_left]->getPoint()[2];

        temp[9] = skeleton[KeyPointTag::elbow_left]->getPoint()[0];
        temp[10] = skeleton[KeyPointTag::elbow_left]->getPoint()[1];
        temp[11] = skeleton[KeyPointTag::elbow_left]->getPoint()[2];

        temp[12] = skeleton[KeyPointTag::hand_left]->getPoint()[0];
        temp[13] = skeleton[KeyPointTag::hand_left]->getPoint()[1];
        temp[14] = skeleton[KeyPointTag::hand_left]->getPoint()[2];

        temp[15] = skeleton[KeyPointTag::shoulder_right]->getPoint()[0];
        temp[16] = skeleton[KeyPointTag::shoulder_right]->getPoint()[1];
        temp[17] = skeleton[KeyPointTag::shoulder_right]->getPoint()[2];

        temp[18] = skeleton[KeyPointTag::elbow_right]->getPoint()[0];
        temp[19] = skeleton[KeyPointTag::elbow_right]->getPoint()[1];
        temp[20] = skeleton[KeyPointTag::elbow_right]->getPoint()[2];

        temp[21] = skeleton[KeyPointTag::hand_right]->getPoint()[0];
        temp[22] = skeleton[KeyPointTag::hand_right]->getPoint()[1];
        temp[23] = skeleton[KeyPointTag::hand_right]->getPoint()[2];

        temp[24] = skeleton[KeyPointTag::hip_left]->getPoint()[0];
        temp[25] = skeleton[KeyPointTag::hip_left]->getPoint()[1];
        temp[26] = skeleton[KeyPointTag::hip_left]->getPoint()[2];

        temp[27] = skeleton[KeyPointTag::hip_center]->getPoint()[0];
        temp[28] = skeleton[KeyPointTag::hip_center]->getPoint()[1];
        temp[29] = skeleton[KeyPointTag::hip_center]->getPoint()[2];

        temp[30] = skeleton[KeyPointTag::knee_left]->getPoint()[0];
        temp[31] = skeleton[KeyPointTag::knee_left]->getPoint()[1];
        temp[32] = skeleton[KeyPointTag::knee_left]->getPoint()[2];

        temp[33] = skeleton[KeyPointTag::ankle_left]->getPoint()[0];
        temp[34] = skeleton[KeyPointTag::ankle_left]->getPoint()[1];
        temp[35] = skeleton[KeyPointTag::ankle_left]->getPoint()[2];

        temp[36] = skeleton[KeyPointTag::hip_right]->getPoint()[0];
        temp[37] = skeleton[KeyPointTag::hip_right]->getPoint()[1];
        temp[38] = skeleton[KeyPointTag::hip_right]->getPoint()[2];

        temp[39] = skeleton[KeyPointTag::knee_right]->getPoint()[0];
        temp[40] = skeleton[KeyPointTag::knee_right]->getPoint()[1];
        temp[41] = skeleton[KeyPointTag::knee_right]->getPoint()[2];

        temp[42] = skeleton[KeyPointTag::ankle_right]->getPoint()[0];
        temp[43] = skeleton[KeyPointTag::ankle_right]->getPoint()[1];
        temp[44] = skeleton[KeyPointTag::ankle_right]->getPoint()[2];
    }

    double getPeriod() override
    {
        return 0.01;
    }

    void updateSkels()
    {
        double t=Time::now()-t0;
        double theta1=2.0*M_PI*0.1*t+phi;

        Vector c(skeleton1[KeyPointTag::shoulder_right]->getPoint());
        Vector pe1=skeleton1[KeyPointTag::elbow_right]->getPoint();
        Vector ph1=skeleton1[KeyPointTag::hand_right]->getPoint();

        pe1[0]=-radius*cos(theta1+phi);
        if(pe1[0] > 0.0)
            pe1[0]=-pe1[0];
        pe1[0]+=c[0];
        pe1[1]=c[1]-radius*fabs(sin(theta1+phi));
        pe1[2]=c[2];

        ph1[0]=-2*radius*cos(theta1+phi);
        if(ph1[0] > 0.0)
            ph1[0]=-ph1[0];
        ph1[0]+=c[0];
        ph1[1]=c[1]-2*radius*fabs(sin(theta1+phi));
        ph1[2]=c[2];

        vector<pair<string,Vector>> unordered1=skeleton1.get_unordered();
        unordered1.push_back(make_pair(KeyPointTag::elbow_right,pe1));
        unordered1.push_back(make_pair(KeyPointTag::hand_right,ph1));
        skeleton1.update(unordered1);

        double theta2=amplitude*2.0*M_PI*0.1*t+(phi+phase);
        Vector c2(skeleton2[KeyPointTag::shoulder_right]->getPoint());
        Vector pe2=skeleton2[KeyPointTag::elbow_right]->getPoint();
        Vector ph2=skeleton2[KeyPointTag::hand_right]->getPoint();

        pe2[0]=-radius*cos(theta2+phi);
        if(pe2[0] > 0.0)
            pe2[0]=-pe2[0];
        pe2[0]+=c2[0];
        pe2[1]=c2[1]-radius*fabs(sin(theta2+phi));
        pe2[2]=c2[2];

        ph2[0]=-2*radius*cos(theta2+phi);
        if(ph2[0] > 0.0)
            ph2[0]=-ph2[0];
        ph2[0]+=c2[0];
        ph2[1]=c2[1]-2*radius*fabs(sin(theta2+phi));
        ph2[2]=c2[2];

        vector<pair<string,Vector>> unordered2=skeleton2.get_unordered();
        unordered2.push_back(make_pair(KeyPointTag::elbow_right,pe2));
        unordered2.push_back(make_pair(KeyPointTag::hand_right,ph2));
        skeleton2.update(unordered2);

        Vector temp1,temp2;
        temp1.resize(3*skeleton1.getNumKeyPoints());
        temp2.resize(3*skeleton2.getNumKeyPoints());

        createVec(skeleton1,temp1);
        createVec(skeleton2,temp2);

        skeleton_template.push_back(temp1);
        skeleton_candidate.push_back(temp2);

        if(skeleton_template.size()>nsamples)
            skeleton_template.erase(skeleton_template.begin());

        if(skeleton_candidate.size()>nsamples)
            skeleton_candidate.erase(skeleton_candidate.begin());

    }

    bool updateModule() override
    {
        updateSkels();

        if(align)
        {
            dtw.update(skeleton_template,skeleton_candidate);
            aligned_skeleton = dtw.align();
            vector<pair<string,Vector>> unordered; //=skeleton2.get_unordered();
            Vector p=aligned_skeleton.back();
            updateUnordered(unordered,p);
            alignedSkel.update(unordered);
        }

        Property prop1=skeleton1.toProperty();
        Property prop2=skeleton2.toProperty();
        Property prop3=alignedSkel.toProperty();

        Bottle &msg=port_out.prepare();
        msg.clear();
        msg.addList().read(prop1);
        msg.addList().read(prop2);
        msg.addList().read(prop3);
        port_out.write();

        double t1=computeMetric(skeleton1);
        double t2=computeMetric(skeleton2);

        t1series.push_back(t1);
        t2series.push_back(t2);
        if(t1series.size()>nsamples)
            t1series.erase(t1series.begin());

        if(t2series.size()>nsamples)
            t2series.erase(t2series.begin());

        dtw.update(t1series,t2series);
        tAligned = dtw.alignSeries();

        Bottle &b=scopePort.prepare();
        b.clear();
        b.addDouble(t1series.back());
        b.addDouble(t2series.back());
        b.addDouble(tAligned.back());
        scopePort.write();

        return true;
    }

    void updateUnordered(vector<pair<string,Vector>>& unordered, const Vector& p)
    {
        unordered.push_back(make_pair(KeyPointTag::shoulder_center,p.subVector(0,2)));
        unordered.push_back(make_pair(KeyPointTag::head,p.subVector(3,5)));
        unordered.push_back(make_pair(KeyPointTag::shoulder_left,p.subVector(6,8)));
        unordered.push_back(make_pair(KeyPointTag::elbow_left,p.subVector(9,11)));
        unordered.push_back(make_pair(KeyPointTag::hand_left,p.subVector(12,14)));
        unordered.push_back(make_pair(KeyPointTag::shoulder_right,p.subVector(15,17)));
        unordered.push_back(make_pair(KeyPointTag::elbow_right,p.subVector(18,20)));
        unordered.push_back(make_pair(KeyPointTag::hand_right,p.subVector(21,23)));
        unordered.push_back(make_pair(KeyPointTag::hip_left,p.subVector(24,26)));
        unordered.push_back(make_pair(KeyPointTag::hip_center,p.subVector(27,29)));
        unordered.push_back(make_pair(KeyPointTag::knee_left,p.subVector(30,32)));
        unordered.push_back(make_pair(KeyPointTag::ankle_left,p.subVector(33,35)));
        unordered.push_back(make_pair(KeyPointTag::hip_right,p.subVector(36,38)));
        unordered.push_back(make_pair(KeyPointTag::knee_right,p.subVector(39,41)));
        unordered.push_back(make_pair(KeyPointTag::ankle_right,p.subVector(42,44)));
    }

    double computeMetric(const SkeletonWaist &s)
    {
        Vector v1(3,0.0), plane_normal(3,0.0),ref_dir(3,0.0);
        plane_normal[2]=1.0;
        ref_dir[1]=-1.0;

        //get reference keypoint from skeleton
        Vector kp_ref = s[KeyPointTag::shoulder_right]->getPoint();
        Vector kp_child = s[KeyPointTag::shoulder_right]->getChild(0)->getPoint();
        v1 = kp_child-kp_ref;

        double dist = dot(v1,plane_normal);
        v1 = v1-dist*plane_normal;

        double n1 = norm(v1);
        if(n1 > 0.0)
            v1 /= n1;
        double n2 = norm(ref_dir);
        double dot_p = dot(v1,ref_dir);

        double theta = acos(dot_p/n2);
        double result = theta * (180/M_PI);
        return result;
    }


    bool close() override
    {
//        file_template.close();
//        file_skeleton.close();
        port_out.close();
        scopePort.close();
        return true;
    }
};

int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    TestDtw testdtw;

    return testdtw.runModule(rf);
}
