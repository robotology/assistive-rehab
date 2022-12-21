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

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogComponent.h>

#include <condition_variable>

#include <mutex>
#include <cmath>

#include <AssistiveRehab/skeleton.h>

#include "src/managerTUG_IDL.h"

namespace
{
    YARP_LOG_COMPONENT(MANAGERTUG, "managerTUG")
}

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace assistive_rehab;

int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("managerTUG");
    rf.setDefaultConfigFile("config-it.ini");
    rf.configure(argc,argv);

    // Manager manager;
    // return manager.runModule(rf);
}
