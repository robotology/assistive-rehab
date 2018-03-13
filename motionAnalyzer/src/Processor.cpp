#include "Processor.h"

using namespace std;
using namespace yarp::math;

Processor::Processor(Vector &elbowLeft_init_, Vector &elbowRight_init_, Vector &handLeft_init_, Vector &handRight_init_,
                     Vector &head_init_, Vector &shoulderCenter_init_, Vector &shoulderLeft_init_, Vector &shoulderRight_init_,
                     Vector &hipLeft_init_, Vector &hipRight_init_, Vector &kneeLeft_init_, Vector &kneeRight_init_)
{
    elbowLeft_init = elbowLeft_init_;
    elbowRight_init = elbowRight_init_;
    handLeft_init = handLeft_init_;
    handRight_init = handRight_init_;
    head_init = head_init_;
    shoulderCenter_init = shoulderCenter_init_;
    shoulderLeft_init = shoulderLeft_init_;
    shoulderRight_init = shoulderRight_init_;
    hipLeft_init = hipLeft_init_;
    hipRight_init = hipRight_init_;
    kneeLeft_init = kneeLeft_init_;
    kneeRight_init = kneeRight_init_;
}

/********************************************************/
Rom_Processor::Rom_Processor(Rom *rom_)
{
    rom = rom_;
}


bool Rom_Processor::checkDeviationFromIntialPose()
{
    //if joint is stationary
    yInfo() << elbowLeft_init[0] << elbowLeft_init[1] << elbowLeft_init[2];

    if(rom->elbowLeft[3] == 0)
    {
//        if(norm(rom->elbowLeft.subVector(0,2) - elbowLeft_init) > 1)
//        {
//            yInfo() << "elbow left deviating from initial pose";
//        }
    }

//    if(rom->elbowRight[3] == 0)
//    {
//        if(norm(rom->elbowRight.subVector(0,2) - elbowRight_init) > 1)
//        {
//            yInfo() << "elbow right deviating from initial pose";
//        }
//    }

//    if(rom->handLeft[3] == 0)
//    {
//        if(norm(rom->handLeft.subVector(0,2) - handLeft_init) > 1)
//        {
//            yInfo() << "hand left deviating from initial pose";
//        }
//    }

//    if(rom->handRight[3] == 0)
//    {
//        if(norm(rom->handRight.subVector(0,2) - handRight_init) > 1)
//        {
//            yInfo() << "hand right deviating from initial pose";
//        }
//    }

//    if(rom->head[3] == 0)
//    {
//        if(norm(rom->head.subVector(0,2) - head_init) > 1)
//        {
//            yInfo() << "head deviating from initial pose";
//        }
//    }

//    if(rom->shoulderCenter[3] == 0)
//    {
//        if(norm(rom->shoulderCenter.subVector(0,2) - shoulderCenter_init) > 1)
//        {
//            yInfo() << "shoulder center deviating from initial pose";
//        }
//    }

//    if(rom->shoulderLeft[3] == 0)
//    {
//        if(norm(rom->shoulderLeft.subVector(0,2) - shoulderLeft_init) > 1)
//        {
//            yInfo() << "shoulder left deviating from initial pose";
//        }
//    }

//    if(rom->shoulderRight[3] == 0)
//    {
//        if(norm(rom->shoulderRight.subVector(0,2) - shoulderRight_init) > 1)
//        {
//            yInfo() << "shoulder right deviating from initial pose";
//        }
//    }

//    if(rom->hipLeft[3] == 0)
//    {
//        if(norm(rom->hipLeft.subVector(0,2) - hipLeft_init) > 1)
//        {
//            yInfo() << "hip left deviating from initial pose";
//        }
//    }

//    if(rom->hipRight[3] == 0)
//    {
//        if(norm(rom->hipRight.subVector(0,2) - hipRight_init) > 1)
//        {
//            yInfo() << "hip right deviating from initial pose";
//        }
//    }

//    if(rom->kneeLeft[3] == 0)
//    {
//        if(norm(rom->kneeLeft.subVector(0,2) - kneeLeft_init) > 1)
//        {
//            yInfo() << "knee left deviating from initial pose";
//        }
//    }

//    if(rom->kneeRight[3] == 0)
//    {
//        if(norm(rom->kneeRight.subVector(0,2) - kneeRight_init) > 1)
//        {
//            yInfo() << "knee right deviating from initial pose";
//        }
//    }

    return true;

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
