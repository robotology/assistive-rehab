#ifndef __EXTRACTOR_H__
#define __EXTRACTOR_H__

#include <iostream>

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <math.h>

using namespace yarp::sig;

class Extractor
{
    Matrix joint_mat;

public:

    Extractor() {;}
    void configure(Matrix &joint_mat_);
    double computeRom();

};

#endif
