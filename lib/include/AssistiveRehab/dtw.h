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

namespace assistive_rehab
{

class Dtw
{
protected:
    int win;
    double d;   //!< dtw distance

    yarp::sig::Matrix initialize(const int ns, const int nt) const;
    void getWarpingPath(const yarp::sig::Matrix &distMat, std::vector<int> &ws, std::vector<int> &wt) const;
    double computeDistance(const std::vector<double> &s, const std::vector<double> &t,
                           yarp::sig::Matrix &distMat) const;

public:
    Dtw();
    Dtw(const int &win_);
    void align(const std::vector<double> &s, const std::vector<double> &t,
               std::vector<double> &ws, std::vector<double> &wt);
    void align(const std::vector<std::vector<double>> &s, const std::vector<std::vector<double>> &t,
               std::vector<std::vector<double> > &ws, std::vector<std::vector<double> > &wt);
    double getDistance() const { return d; }
    virtual ~Dtw() { }
};

}

#endif
