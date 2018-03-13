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

public:
    Processor() {;}
    Processor(Vector &elbowLeft_init_, Vector &elbowRight_init_, Vector &handLeft_init_, Vector &handRight_init_,
              Vector &head_init_, Vector &shoulderCenter_init_,  Vector &shoulderLeft_init_, Vector &shoulderRight_init_,
              Vector &hipLeft_init_, Vector &hipRight_init_, Vector &kneeLeft_init_, Vector &kneeRight_init_);

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

};

class Rom_Processor : public Processor
{

    Rom *rom;

public:

    Rom_Processor(Rom *rom_);
    bool checkDeviationFromIntialPose();
    double computeRom();

};

#endif
