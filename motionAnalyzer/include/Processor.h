#ifndef __PROCESSOR_H__
#define __PROCESSOR_H__

#include <iostream>

#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <math.h>

#include <Metric.h>

using namespace std;
using namespace yarp::sig;

class Processor
{
protected:
    Vector elbowLeft_init;
    Vector elbowRight_init;
    Vector handLeft_init;
    Vector handRight_init;
    Vector head_init;
    Vector shoulderCenter_init;
    Vector shoulderLeft_init;
    Vector shoulderRight_init;
    Vector hipLeft_init;
    Vector hipRight_init;
    Vector kneeLeft_init;
    Vector kneeRight_init;

public:
    Processor() {;}
    Processor(Vector &elbowLeft_, Vector &elbowRight_, Vector &handLeft_, Vector &handRight_,
              Vector &head_, Vector &shoulderCenter_,  Vector &shoulderLeft_, Vector &shoulderRight_,
              Vector &hipLeft_, Vector &hipRight_, Vector &kneeLeft_, Vector &kneeRight_);

};

class Rom_Processor : public Processor
{

    Rom *rom;

public:

    Rom_Processor();
    bool checkDeviationFromIntialPose(Vector &elbowLeft_, Vector &elbowRight_, Vector &handLeft_,
                                      Vector &handRight_, Vector &head_, Vector &shoulderCenter_,
                                      Vector &shoulderLeft_, Vector &shoulderRight_, Vector &hipLeft_,
                                      Vector &hipRight_, Vector &kneeLeft_, Vector &kneeRight_);
    double computeRom();

};

#endif
