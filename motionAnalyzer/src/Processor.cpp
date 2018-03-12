#include "Processor.h"

using namespace std;
using namespace yarp::math;

Processor::Processor(Vector &elbowLeft_, Vector &elbowRight_, Vector &handLeft_, Vector &handRight_,
                     Vector &head_, Vector &shoulderCenter_, Vector &shoulderLeft_, Vector &shoulderRight_,
                     Vector &hipLeft_, Vector &hipRight_, Vector &kneeLeft_, Vector &kneeRight_)
{
    elbowLeft_init = elbowLeft_;
    elbowRight_init = elbowRight_;
    handLeft_init = handLeft_;
    handRight_init = handRight_;
    head_init = head_;
    shoulderCenter_init = shoulderCenter_;
    shoulderLeft_init = shoulderLeft_;
    shoulderRight_init = shoulderRight_;
    hipLeft_init = hipLeft_;
    hipRight_init = hipRight_;
    kneeLeft_init = kneeLeft_;
    kneeRight_init = kneeRight_;
}

/********************************************************/
Rom_Processor::Rom_Processor()
{

}


bool Rom_Processor::checkDeviationFromIntialPose(Vector &elbowLeft_, Vector &elbowRight_, Vector &handLeft_, Vector &handRight_,
                                                 Vector &head_, Vector &shoulderCenter_, Vector &shoulderLeft_, Vector &shoulderRight_,
                                                 Vector &hipLeft_, Vector &hipRight_, Vector &kneeLeft_, Vector &kneeRight_)
{

}

double Rom_Processor::computeRom()
{

    //    if(j1.size() && j2.size() && j3.size())
    //    {
    //        double a_norm = norm(j1-j2);
    //        double b_norm = norm(j1-j3);

    //        double dot_p = dot(j1-j2, j1-j3);

    //        return ( acos(dot_p/(a_norm*b_norm)) * (180/M_PI) );
    //    }
    //    else
    //        return 0.0;

}
