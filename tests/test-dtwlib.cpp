/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-dtwlib.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <cstdlib>
#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include "AssistiveRehab/dtw.h"

using namespace std;
using namespace assistive_rehab;

int main()
{
    ofstream outfile;
    outfile.open("dtwlib-test.txt");

    /*****************************/
    /*    Different frequency    */
    /*****************************/
    cout << "### Generating 2 sinusoidal 1D vectors at different frequencies ###" << endl;
    int n1=100, n2=100;
    double f1=0.1, f2=0.8;
    double A1=1.0, A2=1.0;
    vector<double> v1_1(n1);
    vector<double> v2_1(n2);

    cout << "f1 = " << f1 << " f2 = " << f2 << endl;

    double t=0.0;
    for(int i=0; i<n1; i++)
    {
        v1_1[i]=A1*sin(2.0*M_PI*f1*t);
//        outfile << v1[i] << " ";
        t+=0.1;
    }
//    outfile << "\n";

    t=0.0;
    for(int i=0; i<n2; i++)
    {
        v2_1[i]=A2*sin(2.0*M_PI*f2*t);
//        outfile << v2[i] << " ";
        t+=0.1;
    }
//    outfile << "\n";

    cout << "### Aligning vectors ###" << endl;
    Dtw dtw1(-1);
    vector<double> w_v1_1,w_v2_1;
    dtw1.align(v1_1,v2_1,w_v1_1,w_v2_1);

    double d1 = dtw1.getDistance();
    cout << "DTW distance = " << d1 << endl;
    cout << endl;

    /*****************************/
    /*     Close frequencies     */
    /*****************************/
    cout << "### Generating 2 sinusoidal 1D vectors at close frequencies ###" << endl;
    f1=0.7, f2=0.8;
    vector<double> v1_2(n1);
    vector<double> v2_2(n2);

    cout << "f1= " << f1 << " f2 = " << f2 << endl;

    t=0.0;
//    cout << "v1: ";
    for(int i=0; i<n1; i++)
    {
        v1_2[i]=A1*sin(2.0*M_PI*f1*t);
//        outfile << v1[i] << " ";
        t+=0.1;
    }
//    outfile << "\n";

    t=0.0;
    for(int i=0; i<n2; i++)
    {
        v2_2[i]=A2*sin(2.0*M_PI*f2*t);
//        outfile << v2[i] << " ";
        t+=0.1;
    }
//    outfile << "\n";

    cout << "### Aligning vectors ###" << endl;
    Dtw dtw2(-1);
    vector<double> w_v1_2,w_v2_2;
    dtw2.align(v1_2,v2_2,w_v1_2,w_v2_2);

    double d2 = dtw2.getDistance();
    cout << "DTW distance = " << d2 << endl;
    cout << endl;

    cout << "### Generating 2 sinusoidal n-D vectors ###" << endl;

    int n=6;
    cout << "f1 = " << f1 << " f2 = " << f2 << " n = " << n << endl;
    vector<vector<double>> v1_3d(n,vector<double>(n1));
    vector<vector<double>> v2_3d(n,vector<double>(n2));

    t=0.0;
    for(int i=0; i<n; i++)
    {
        for(int j=0; j<n1; j++)
        {
            v1_3d[i][j]=A1*sin(2.0*M_PI*f1*t);
            outfile << v1_3d[i][j] << " ";
            t+=0.1;
        }
        outfile << "\n";
    }
    outfile << "\n";

    t=0.0;
    for(int i=0; i<n; i++)
    {
        for(int j=0; j<n2; j++)
        {
            v2_3d[i][j]=A2*sin(2.0*M_PI*f2*t);
            outfile << v2_3d[i][j] << " ";
            t+=0.1;
        }
        outfile << "\n";
    }
    outfile << "\n";

    cout << "### Aligning vectors ###" << endl;
    Dtw dtw_3d(-1);
    vector<vector<double>> w_v1_3d,w_v2_3d;
    dtw_3d.align(v1_3d,v2_3d,w_v1_3d,w_v2_3d);
    double d_3d = dtw_3d.getDistance();
    cout << "DTW distance = " << d_3d << endl;
    cout << endl;

    outfile.close();

    return EXIT_SUCCESS;
}
