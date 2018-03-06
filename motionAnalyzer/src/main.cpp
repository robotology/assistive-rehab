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

#include "motionAnalyzer_IDL.h"

using namespace std;
using namespace yarp::os;


/********************************************************/
class Manager : public RFModule,
                public motionAnalyzer_IDL
{


    /********************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:
    /********************************************************/
    bool configure(ResourceFinder &rf)
    {

        return true;
    }

    /********************************************************/
    bool close()
    {
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /********************************************************/
    bool updateModule()
    {
        return true;
    }

};

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
    rf.configure(argc, argv);

    return manager.runModule(rf);

    return 0;
}
