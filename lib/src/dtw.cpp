/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file dtw.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <iostream>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include "AssistiveRehab/dtw.h"

using namespace std;
using namespace yarp::sig;

Dtw::Dtw() : win(-1), d(0.0)
{
}

Dtw::Dtw (const int &win_) : Dtw()
{
    win = win_;
}

Matrix Dtw::initialize(const int ns, const int nt) const
{
    Matrix distMat(ns+1,nt+1);

    //initialize distance matrix
    for(int i=0;i<ns+1;i++)
    {
        for(int j=0;j<nt+1;j++)
        {
            distMat[i][j]=-1;
        }
    }
    distMat[0][0]=0;
    return distMat;
}

double Dtw::computeDistance(const vector<double> &s, const vector<double> &t, Matrix &distMat) const
{
    //compute distance matrix
    int ns=(int)s.size();
    int nt=(int)t.size();
    int j1,j2;
    double cost,temp;
    for(int i=1;i<=ns;i++)
    {
        if(win==-1)
        {
            j1=1;
            j2=nt;
        }
        else
        {
            j1= i-win>1 ? i-win : 1;
            j2= i+win<nt ? i+win : nt;
        }
        for(int j=j1;j<=j2;j++)
        {
            cost=sqrt((s[i-1]-t[j-1])*(s[i-1]-t[j-1]));
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

    return(distMat[ns][nt]/nt);
}

void Dtw::getWarpingPath(const Matrix &distMat, vector<int> &ws, vector<int> &wt) const
{
    int m=(int)distMat.rows()-1;
    int n=(int)distMat.cols()-1;
    double temp;

    ws.push_back(m);
    wt.push_back(n);
    while( (n+m) != 2 )
    {
        if( (n-1)==0 )
        {
            m=m-1;
        }
        else if ( (m-1)==0 )
        {
            n=n-1;
        }
        else
        {
            temp=distMat[m-1][n];
            int c=0;
            if(distMat[m][n-1]<temp)
            {
                temp=distMat[m][n-1];
                c=1;
            }
            if(distMat[m-1][n-1]<temp)
            {
                temp=distMat[m-1][n-1];
                c=2;
            }

            switch (c)
            {
            case 0:
                m=m-1;
                break;
            case 1:
                n=n-1;
                break;
            case 2:
                m=m-1;
                n=n-1;
                break;
            }
        }

        ws.push_back(m);
        wt.push_back(n);
    }

}

/***************************/
/*  Mono-dimensional DTW   */
/***************************/
void Dtw::align(const vector<double> &s, const vector<double> &t, vector<double> &ws, vector<double> &wt)
{
    int ns=(int)s.size();
    int nt=(int)t.size();

    //create and initialize distance matrix
    Matrix distMat = initialize(ns,nt);

    //compute distance matrix
    d=computeDistance(s,t,distMat);

    //align
    vector<int> w1,w2;
    getWarpingPath(distMat,w1,w2);
    ws.clear();
    wt.clear();
    for(int i=(int)w1.size()-1;i>=0;i--)
    {
        ws.push_back(s[w1[i]-1]);
    }
    for(int i=(int)w2.size()-1;i>=0;i--)
    {
        wt.push_back(t[w2[i]-1]);
    }
}

/***************************/
/*  Multi-dimensional DTW  */
/***************************/
void Dtw::align(const vector<vector<double>> &s, const vector<vector<double>> &t,
                vector<vector<double>> &ws, vector<vector<double>> &wt)
{
    int n=(int)s.size(); //number of components
    int ns=(int)s[0].size(); //number of samples of the first vector
    int nt=(int)t[0].size(); //number of samples of the second vector
    ws.resize(n);
    wt.resize(n);

    //compute distance matrix for each component of the vectors
    //vectors must have the same number of components
    for(int l=0; l<n; l++)
    {
        //create and initialize distance matrix
        Matrix distMat = initialize(ns,nt);

        //compute distance
        d+=computeDistance(s[l],t[l],distMat);

        //align
        vector<int> w1,w2;
        getWarpingPath(distMat,w1,w2);
        ws[l].clear();
        wt[l].clear();
        for(int i=(int)w1.size()-1;i>=0;i--)
        {
            ws[l].push_back(s[l][w1[i]-1]);
        }
        for(int i=(int)w2.size()-1;i>=0;i--)
        {
            wt[l].push_back(t[l][w2[i]-1]);
        }
    }
}

