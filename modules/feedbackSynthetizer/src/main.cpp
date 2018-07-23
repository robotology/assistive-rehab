/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

#include <cstdlib>
#include <yarp/os/all.h>
#include "AssistiveRehab/skeleton.h"

using namespace std;
using namespace yarp::os;
using namespace assistive_rehab;

class Synthetizer : public RFModule
{

public:

    bool configure(ResourceFinder &rf) override
    {

    }

    double getPeriod() override
    {
        return 0.1;
    }

    bool updateModule() override
    {

    }

    bool interruptModule() override
    {

    }

    bool close() override
    {

    }

};

int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Synthetizer feed_synthetizer;

    return feed_synthetizer.runModule(rf);
}
