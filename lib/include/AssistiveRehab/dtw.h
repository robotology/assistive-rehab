/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file dtw.h
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#ifndef ASSISTIVE_REHAB_DTW_H
#define ASSISTIVE_REHAB_DTW_H

#include <vector>
#include <yarp/sig/Matrix.h>

class Dtw
{
private:

    //parameters
    int win;

    //dtw distance
    double d;

    yarp::sig::Matrix initialize(const int ns, const int nt);
    int getMin(yarp::sig::Matrix &distMat, const int row, const int nt);
    double computeDistance(const std::vector<double> &s, const std::vector<double> &t, yarp::sig::Matrix &distMat);

public:

    Dtw();
    Dtw(const int &win_);
    std::vector<double> align(const std::vector<double> &s, const std::vector<double> &t);
    std::vector<std::vector<double>> align(const std::vector<std::vector<double>> &s,
                                           const std::vector<std::vector<double>> &t);
    double getDistance() const { return d; }
    ~Dtw();

};

#endif
