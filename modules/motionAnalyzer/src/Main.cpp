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

#include <yarp/os/all.h>

#include "Manager.h"

using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;
    if(!yarp.checkNetwork())
    {
        yError() << "YARP server not available";
        return -1;
    }

    Manager manager;
    ResourceFinder rf;

    rf.setDefaultContext("motionAnalyzer");
    rf.setDefaultConfigFile("motion-repertoire.ini");
    rf.configure(argc, argv);

    return manager.runModule(rf);
}
