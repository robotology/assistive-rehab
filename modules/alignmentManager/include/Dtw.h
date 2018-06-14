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

#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

using namespace std;
using namespace yarp::sig;

class Dtw
{
private:
    double** distMat;
    int ns,nt,w,k;
    vector<Vector> s,t;
    vector<double> s1,s2;

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

    void computeDistance(const vector<double> &s1, const vector<double> &s2)
    {
        int j1,j2;
        double cost,temp;

        for(int i=0;i<s1.size();i++)
        {
            if(w==-1)
            {
                j1=0;
                j2=s2.size();
            }
            else
            {
                j1= i-w>1 ? i-w : 0;
                j2= i+w<s1.size() ? i+w : s2.size();
            }
            for(int j=j1;j<j2;j++)
            {
                cost=vectorDistanceSeries(s1[i],s2[j]);

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

    double vectorDistanceSeries(const double &s1, const double &s2)
    {
        return sqrt((s1-s2)*(s1-s2));
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

    void update(const vector<double> &s1_, const vector<double> &s2_)
    {
        s1 = s1_;
        s2 = s2_;
    }

    vector<Vector> align()
    {
        vector<Vector> res;
        computeDistance(s,t);

        for(int i=0;i<s.size();i++)
        {
            int j=findMin(i);
            res.push_back(t[j]);
        }
        return res;
    }

    vector<double> alignSeries()
    {
        vector<double> res;
        computeDistance(s1,s2);

        for(int i=0;i<s1.size();i++)
        {
            int j=findMin(i);
            res.push_back(s2[j]);
        }
        return res;
    }

    ~Dtw()
    {
        for(int i=0;i<s.size();i++)
        {
            delete distMat[i];
        }
        delete distMat;
    }

};
