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

Dtw::Dtw() : win(-1)
{
    d = 0.0;
}

Dtw::Dtw (const int &win_)
{
    win = win_;
    d = 0.0;
}

Matrix Dtw::initialize(const int ns, const int nt)
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

double Dtw::computeDistance(const vector<double> &s, const vector<double> &t, Matrix &distMat)
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

int Dtw::getMin(Matrix &distMat, const int row, const int nt)
{
    int col=1;
    double mind=distMat[row][col];
    for(int j=1;j<nt+1;j++)
    {
        if(distMat[row][j]<mind)
        {
            mind=distMat[row][j];
            col=j;
        }
    }
    return col;
}

/***************************/
/*  Mono-dimensional DTW   */
/***************************/
vector<double> Dtw::align(const vector<double> &s, const vector<double> &t)
{
    int ns=(int)s.size();
    int nt=(int)t.size();

    //create and initialize distance matrix
    Matrix distMat = initialize(ns,nt);

    //compute distance matrix
    d=computeDistance(s,t,distMat);

    //align
    vector<double> res;
    res.resize(ns);
    for(int i=1;i<ns+1;i++)
    {
        int j=getMin(distMat,i,nt);
        res[i-1]=t[j-1];
    }

    return res;
}

/***************************/
/*  Multi-dimensional DTW  */
/***************************/
vector<vector<double>> Dtw::align(const vector<vector<double>> &s, const vector<vector<double>> &t)
{
    int n=(int)s.size();
    int ns=(int)s[0].size();
    int nt=(int)t[0].size();
    vector<vector<double>> res(n,vector<double>(nt));

    //compute distance matrix for each component of the vectors
    //vectors must have the same number of components
    for(int l=0; l<n; l++)
    {
        //create and initialize distance matrix
        Matrix distMat = initialize(ns,nt);

        //compute distance
        d+=computeDistance(s[l],t[l],distMat);

        //align
        for(int k=1;k<ns+1;k++)
        {
            int j=getMin(distMat,k,nt);
            res[l][k-1]=t[l][j-1];
        }
    }

    return res;
}

Dtw::~Dtw()
{

}
