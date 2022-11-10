/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
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

#include <vector>
#include <iostream>
#include <deque>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <ctime>

#include <fstream>
#include <iterator>
#include <string>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/SoundFile.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/SoundFile.h>

#include "googleSpeech_IDL.h"

class FakeProcessing
{
    std::string moduleName;
    yarp::os::RpcServer handlerPort;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> questionPort;
    yarp::os::BufferedPort<yarp::os::Bottle> statusPort;
    yarp::os::RpcClient audioCommand;
    yarp::os::Mutex mutex;

    std::deque<yarp::os::Bottle> sounds;

    int samples;
    int channels;
    int padding;
    bool getSounds;
    bool sendForQuery;
    std::string language;
    int sample_rate;

    std::chrono::time_point<std::chrono::system_clock> start, end;

public:
    /********************************************************/

    FakeProcessing( const std::string &moduleName, const std::string &language, const int sample_rate )
    {
        this->moduleName = moduleName;
        yInfo() << "language " << language;
        yInfo() << "sample_rate " << sample_rate;

        this->language = language;
        this->sample_rate = sample_rate;

        port.setStrict();
        samples = 0;
        channels = 0;
        padding = 0;
        getSounds = false;
        sendForQuery = false;
    }

    /********************************************************/
    ~FakeProcessing()
    {};

    /********************************************************/
    bool open()
    {
        port.setStrict(true);

        port.open("/" + moduleName + "/sound:i");
        targetPort.open("/"+ moduleName + "/result:o");
        audioCommand.open("/"+ moduleName + "/commands:rpc");
        questionPort.open("/"+ moduleName + "/question:o");
        statusPort.open("/"+ moduleName + "/status:o");

        return true;
    }

    /********************************************************/
    void close()
    {
        port.close();
        targetPort.close();
        audioCommand.close();
        questionPort.close();
        statusPort.close();
    }
};


/********************************************************/
class FakeModule : public yarp::os::RFModule, public googleSpeech_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    FakeProcessing              *processing;
    friend class                processing;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:
    /********************************************************/
    FakeModule() : closing(false) { }

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("yarp-google-speech"), "module name (string)").asString();

        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool start()
    {
        return true;
    }

    /**********************************************************/
    bool stop()
    {
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /********************************************************/
    bool quit()
    {
        closing=true;
        return true;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
};


/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    yarp::os::ResourceFinder rf;

    rf.setDefaultContext( "googleSpeech" );
    rf.setDefaultConfigFile( "config.ini" );
    rf.setDefault("name","googleSpeech");
    rf.configure(argc,argv);

    FakeModule fakemodule;
    return fakemodule.runModule(rf);
}
