/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Valentina Vasco
 * email:  valentina.vasco@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/all.h>

#include <Manager.h>

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

    rf.setVerbose();
    rf.setDefaultContext("motionAnalyzer");
//    rf.setDefaultConfigFile("motion-list.ini");
    rf.setDefault("configuration-file", "motion-list.ini");

    rf.configure(argc, argv);


    return manager.runModule(rf);

    return 0;
}
