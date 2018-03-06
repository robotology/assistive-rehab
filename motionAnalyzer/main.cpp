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

#include <map>

#include <yarp/os/all.h>

#include "motionAnalyzer_IDL.h"

using namespace std;
using namespace yarp::os;


/********************************************************/
class Manager : public RFModule,
                public motionAnalyzer_IDL
{

    RpcClient opcPort;
    RpcServer rpcPort;

    ResourceFinder *rf;


    /********************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /********************************************************/
    bool load()
    {

        return true;
    }


public:
    /********************************************************/
    bool configure(ResourceFinder &rf)
    {
        this->rf = &rf;
        string moduleName = rf.check("name", Value("motionAnalyzer")).asString();
        setName(moduleName.c_str());

        string robot = rf.check("robot", Value("icub")).asString();

        opcPort.open(("/" + getName() + "/opc").c_str());
        rpcPort.open(("/" + getName() + "/cmd").c_str());
        attach(rpcPort);

        if(!load())
            return false;

        return true;
    }

    /********************************************************/
    bool close()
    {
        opcPort.close();
        rpcPort.close();
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
    rf.setDefaultContext("motionAnalyzer");
    rf.setDefault("configuration-file","motionList.ini");
    rf.configure(argc, argv);

    return manager.runModule(rf);

    return 0;
}
