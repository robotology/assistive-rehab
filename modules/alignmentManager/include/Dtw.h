/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Dtw.h
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <iostream>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::sig;

class Dtw
{
private:
    double** distMat;
    int ns,nt,w;
    vector<Vector> s,t;
    vector<double> s1,s2;

public:

    Dtw() : w(-1),ns(500), nt(500)
    {
    }

    Dtw(const int &w_,const int &ns_,const int &nt_)
    {
        w = w_;
        ns = ns_;
        nt = nt_;

        distMat = new double*[ns+1];
        for(int i=0;i<ns+1;i++)
        {
            distMat[i] = new double[nt+1];
        }
    }

//    void initialize(const int &w_,const int &ns_,const int &nt_,const int &k_)
//    {
//        ns = ns_;
//        nt = nt_;
//        w = w_;
//        k = k_;

//        distMat = new double*[ns];
//        for(int i=0;i<ns;i++)
//        {
//            distMat[i] = new double[nt];
//        }

//        for(int i=0;i<ns;i++)
//        {
//            for(int j=0;j<nt;j++)
//            {
//                distMat[i][j]=-1;
//            }
//        }
//        distMat[0][0]=0;
//    }

//    double computeDistance(const vector<Vector> &s, const vector<Vector> &t)
//    {
//        int j1,j2;
//        double cost,temp;

//        for(int i=0;i<s.size();i++)
//        {
//            if(w==-1)
//            {
//                j1=0;
//                j2=t.size();
//            }
//            else
//            {
//                j1= i-w>1 ? i-w : 0;
//                j2= i+w<t.size() ? i+w : t.size();
//            }
//            for(int j=j1;j<j2;j++)
//            {
//                cost=vectorDistance(s[i],t[j]);

//                if((i-1)>0 && (j-1)>0)
//                {
//                    temp=distMat[i-1][j];
//                    if(distMat[i][j-1]!=-1)
//                    {
//                        if(temp==-1 || distMat[i][j-1]<temp) temp=distMat[i][j-1];
//                    }
//                    if(distMat[i-1][j-1]!=-1)
//                    {
//                        if(temp==-1 || distMat[i-1][j-1]<temp) temp=distMat[i-1][j-1];
//                    }
//                }
//                else
//                    temp = 0.0;

//                distMat[i][j]=cost+temp;

//            }
//        }
//        return distMat[ns-1][nt-1];
//    }

    double computeDistance(const vector<double> &s1, const vector<double> &s2)
    {
        for(int i=0;i<ns+1;i++)
        {
            for(int j=0;j<nt+1;j++)
            {
                distMat[i][j]=-1;
            }
        }
        distMat[0][0]=0;

        int j1,j2;
        double cost,temp;
        for(int i=1;i<=ns;i++)
        {
            if(w==-1)
            {
                j1=1;
                j2=nt;
            }
            else
            {
                j1= i-w>1 ? i-w : 1;
                j2= i+w<nt ? i+w : nt;
            }
            for(int j=j1;j<=j2;j++)
            {
                cost=sqrt((s1[i-1]-s2[j-1])*(s1[i-1]-s2[j-1]));
                temp=distMat[i-1][j];
                if(distMat[i][j-1]!=-1)
                {
                    if(temp==-1 || distMat[i][j-1]<temp)
                        temp=distMat[i][j-1];
                }
                if(distMat[i-1][j-1]!=-1)
                {
                    if(temp==-1 || distMat[i-1][j-1]<temp)
                        temp=distMat[i-1][j-1];
                }

                distMat[i][j]=cost+temp;
            }
        }

//        for(int i=0; i<s1.size(); i++)
//            cout << s1[i] << " ";
//        cout << endl;
//        cout << endl;

//        for(int i=0; i<s2.size(); i++)
//            cout << s2[i] << " ";
//        cout << endl;
//        cout << endl;

//        for(int i=0; i<ns+1; i++)
//        {
//            for(int j=0; j<nt+1; j++)
//            {
//                cout << distMat[i][j] << " ";
//            }
//            cout << endl;
//        }
//        cout << endl;
//        cout << endl;

        return (distMat[ns][nt])/ns;
    }

//    double vectorDistance(const Vector &s, const Vector &t)
//    {
//        double temp=0.0,result=0.0;
//        double ss,tt;
//        for(int x=0;x<k;x++)
//        {
//            ss=s[x];
//            tt=t[x];
//            temp+=((ss-tt)*(ss-tt));
//            if((x+1)%3==0)
//            {
//                result+=sqrt(temp);
//                temp=0.0;
//            }
//        }
//        return result;
//    }

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

//    void update(const vector<Vector> &s_, const vector<Vector> &t_)
//    {
//        s = s_;
//        t = t_;
//    }

//    void update(const vector<double> &s1_, const vector<double> &s2_)
//    {
//        s1 = s1_;
//        s2 = s2_;
//    }

//    vector<Vector> align()
//    {
//        vector<Vector> res;
//        computeDistance(s,t);

//        for(int i=0;i<s.size();i++)
//        {
//            int j=findMin(i);
//            res.push_back(t[j]);
//        }
//        return res;
//    }

//    vector<double> alignSeries()
//    {
//        vector<double> res;
//        computeDistance(s1,s2);

//        for(int i=0;i<s1.size();i++)
//        {
//            int j=findMin(i);
//            res.push_back(s2[j]);
//        }
//        return res;
//    }

    ~Dtw()
    {
        for(int i=0;i<ns+1;i++)
        {
            delete distMat[i];
        }
        delete distMat;
    }

};
