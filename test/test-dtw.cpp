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
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

class Dtw
{
private:
    double** distMat;
    int ns,nt,w,k;
    vector<Vector> s,t;

public:

    Dtw() : w(5),ns(500), nt(500),k(45)
    {
    }

    void initialize(const int &w_,const int &ns_,const int &nt_,const int &k_)
    {
        ns = ns_;
        nt = nt_;
        w = w_;
        k = k_;

        distMat = new double*[ns];
        for(int i=0;i<ns;i++)
        {
            distMat[i] = new double[nt];
        }

        for(int i=0;i<ns;i++)
        {
            for(int j=0;j<nt;j++)
            {
                distMat[i][j]=-1;
            }
        }
        distMat[0][0]=0;
    }

    void computeDistance(const vector<Vector> &s, const vector<Vector> &t)
    {
        int j1,j2;
        double cost,temp;

        for(int i=0;i<s.size();i++)
        {
            if(w==-1)
            {
                j1=0;
                j2=t.size();
            }
            else
            {
                j1= i-w>1 ? i-w : 0;
                j2= i+w<t.size() ? i+w : t.size();
            }
            for(int j=j1;j<j2;j++)
            {
                cost=vectorDistance(s[i],t[j]);

                if((i-1)>0 && (j-1)>0)
                {
                    temp=distMat[i-1][j];
                    if(distMat[i][j-1]!=-1)
                    {
                        if(temp==-1 || distMat[i][j-1]<temp) temp=distMat[i][j-1];
                    }
                    if(distMat[i-1][j-1]!=-1)
                    {
                        if(temp==-1 || distMat[i-1][j-1]<temp) temp=distMat[i-1][j-1];
                    }
                }
                else
                    temp = 0.0;

                distMat[i][j]=cost+temp;

            }
        }
    }

    double vectorDistance(const Vector &s, const Vector &t)
    {
        double result=0;
        double ss,tt;
        for(int x=0;x<k;x++)
        {
            ss=s[x];
            tt=t[x];
            result+=((ss-tt)*(ss-tt));
        }
        result=sqrt(result);
        return result;
    }

    int findMin(const int &row)
    {
        double mind = 0.0;
        int col = 0;
        for(int j=0;j<t.size();j++)
        {
            if(distMat[row][j] < mind)
            {
                mind = distMat[row][j];
                col = j;
            }
        }
        return col;
    }

    void update(const vector<Vector> &s_, const vector<Vector> &t_)
    {
        s = s_;
        t = t_;
    }

    vector<Vector> align()
    {
        vector<Vector> res;
        computeDistance(s,t);

        for(int i=0;i<s.size();i++)
        {
            int j=findMin(i);
            res.push_back(s[j]);
        }
        return res;
    }

    ~Dtw()
    {
        for(int i=0;i<ns+1;i++)
        {
            delete distMat[i];
        }
        delete distMat;
    }

};

class TestDtw : public RFModule
{
private:
    double n;
    int win;
    bool align;
    SkeletonWaist skeleton1;
    SkeletonWaist skeleton2;
    BufferedPort<Bottle> port_out;
    double radius,phase,t0;
    vector<Vector> skeleton_template,skeleton_candidate,aligned_skeleton;
    int nsamples;
//    ofstream file_template,file_skeleton;
    Dtw dtw;

public:

    bool configure(ResourceFinder &rf) override
    {
//        file_template.open("template.txt");
//        file_skeleton.open("skeleton.txt");

        n = rf.check("n",Value(2.0)).asDouble();
        win = rf.check("win",Value(5)).asDouble();
        align = rf.check("align",Value(true)).asBool();
        nsamples = rf.check("nsamples",Value(500)).asDouble();

        dtw.initialize(win,nsamples,nsamples,45);

        skeleton1.setTag("template");
        skeleton2.setTag("skeleton");
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
        phase=atan2(d[1],d[0]);

        port_out.open("/test-dtw");
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
        double theta1=2.0*M_PI*0.1*t+phase;

        Vector c(skeleton1[KeyPointTag::shoulder_right]->getPoint());
        Vector pe1=skeleton1[KeyPointTag::elbow_right]->getPoint();
        Vector ph1=skeleton1[KeyPointTag::hand_right]->getPoint();

        pe1[0]=-radius*cos(theta1+phase);
        if(pe1[0] > 0.0)
            pe1[0]=-pe1[0];
        pe1[0]+=c[0];
        pe1[1]=c[1]-radius*fabs(sin(theta1+phase));
        pe1[2]=c[2];

        ph1[0]=-2*radius*cos(theta1+phase);
        if(ph1[0] > 0.0)
            ph1[0]=-ph1[0];
        ph1[0]+=c[0];
        ph1[1]=c[1]-2*radius*fabs(sin(theta1+phase));
        ph1[2]=c[2];

        vector<pair<string,Vector>> unordered1=skeleton1.get_unordered();
        unordered1.push_back(make_pair(KeyPointTag::elbow_right,pe1));
        unordered1.push_back(make_pair(KeyPointTag::hand_right,ph1));
        skeleton1.update(unordered1);

        double theta2=n*2.0*M_PI*0.1*t+phase;
        Vector c2(skeleton2[KeyPointTag::shoulder_right]->getPoint());
        Vector pe2=skeleton2[KeyPointTag::elbow_right]->getPoint();
        Vector ph2=skeleton2[KeyPointTag::hand_right]->getPoint();

        pe2[0]=-radius*cos(theta2+phase);
        if(pe2[0] > 0.0)
            pe2[0]=-pe2[0];
        pe2[0]+=c2[0];
        pe2[1]=c2[1]-radius*fabs(sin(theta2+phase));
        pe2[2]=c2[2];

        ph2[0]=-2*radius*cos(theta2+phase);
        if(ph2[0] > 0.0)
            ph2[0]=-ph2[0];
        ph2[0]+=c2[0];
        ph2[1]=c2[1]-2*radius*fabs(sin(theta2+phase));
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

//        file_template << skeleton1[KeyPointTag::hand_right]->getPoint()[0] << " " << skeleton1[KeyPointTag::hand_right]->getPoint()[1]
//                << " " << skeleton1[KeyPointTag::hand_right]->getPoint()[2] << "\n";
//        file_skeleton << skeleton2[KeyPointTag::hand_right]->getPoint()[0] << " " << skeleton2[KeyPointTag::hand_right]->getPoint()[1]
//                << " " << skeleton2[KeyPointTag::hand_right]->getPoint()[2] << "\n";

        if(align)
        {
            dtw.update(skeleton_template,skeleton_candidate);
            aligned_skeleton = dtw.align();
            vector<pair<string,Vector>> unordered=skeleton2.get_unordered();
            Vector p = aligned_skeleton.back();
            updateUnordered(unordered, p);
            skeleton2.update(unordered);
        }

        Property prop1=skeleton1.toProperty();
        Property prop2=skeleton2.toProperty();

        Bottle &msg=port_out.prepare();
        msg.clear();
        msg.addList().read(prop1);
        msg.addList().read(prop2);
        port_out.write();

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

    bool close() override
    {
//        file_template.close();
//        file_skeleton.close();
        port_out.close();
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
