#ifndef __MANAGER_H__
#define __MANAGER_H__

#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <Manager.h>
#include <Processor.h>
#include <Metric.h>

#include "motionAnalyzer_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class Manager : public RFModule,
                public motionAnalyzer_IDL
{

    RpcClient opcPort;
    RpcServer rpcPort;
    BufferedPort<Bottle> scopePort;

    ResourceFinder *rf;

    int nmovements;

//    struct listEntry
//    {
//        string tag;
//        int id_joint;
//        int n_motion;
//        double min;
//        double max;
//    };
//    map<string, listEntry> motion_list;

    map<string,Metric*> motion_list;

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

    Vector elbowLeft;
    Vector elbowRight;
    Vector handLeft;
    Vector handRight;
    Vector head;
    Vector shoulderCenter;
    Vector shoulderLeft;
    Vector shoulderRight;
    Vector hipLeft;
    Vector hipRight;
    Vector kneeLeft;
    Vector kneeRight;

    Processor *processor;

    void init();
    bool load();
    void getKeyframes();
    bool attach(RpcServer &source);

public:

    bool configure(ResourceFinder &rf);
    bool close();
    double getPeriod();
    bool updateModule();

};

#endif
