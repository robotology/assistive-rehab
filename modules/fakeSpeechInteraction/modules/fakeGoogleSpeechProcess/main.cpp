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

#include <fstream>
#include <iterator>
#include <string>
#include <unordered_map>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/SoundFile.h>
#include <yarp/dev/PolyDriver.h>

#include "googleSpeechProcess_IDL.h"

/********************************************************/
class FakeProcessing : public yarp::os::BufferedPort<yarp::os::Bottle>
{
    std::string moduleName;
    std::unordered_map<std::string,std::vector<std::string>> key_map;
    yarp::os::RpcServer handlerPort;
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> statusPort;

    yarp::os::Bottle wordList;

public:
    /********************************************************/

    FakeProcessing( const std::string &moduleName, const std::unordered_map<std::string, std::vector<std::string>> &key_map )
    {
        this->moduleName = moduleName;
        this->key_map = key_map;
    }

    /********************************************************/
    ~FakeProcessing()
    {};

    /********************************************************/
    bool open()
    {
        this->useCallback();
        yarp::os::BufferedPort<yarp::os::Bottle >::open( "/" + moduleName + "/text:i" );
        targetPort.open("/"+ moduleName + "/result:o");
        statusPort.open("/"+ moduleName + "/status:o");

        //yarp::os::Network::connect("/googleSpeech/result:o", 
        //yarp::os::BufferedPort<yarp::os::Bottle >::getName().c_str());

        return true;
    }

    /********************************************************/
    void close()
    {
        yarp::os::BufferedPort<yarp::os::Bottle >::close();
        targetPort.close();
        statusPort.close();
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &bot )
    {
        yarp::os::Bottle &outTargets = targetPort.prepare();

        wordList.clear();
        outTargets = fakeQueryGoogleSyntax(bot);

        targetPort.write();
        yDebug() << "Done FAKE querying google";

    }

    /********************************************************/
    yarp::os::Bottle fakeQueryGoogleSyntax(yarp::os::Bottle& text)
    {
        yDebug() << "in fakeQueryGoogleSyntax";

        yDebug() << "Phrase is " << text.toString().c_str();

        yarp::os::Bottle &outStatus = statusPort.prepare();
        outStatus.clear();
        yDebug() << "Status returned OK";
        yDebug() << "\n------Response------\n";

        outStatus.addString("everything ok");
        statusPort.write();

        yarp::os::Bottle b;
        b.clear();

        return b;
    }

    /********************************************************/
    bool start_acquisition()
    {
        return true;
    }

    /********************************************************/
    bool stop_acquisition()
    {
        return true;
    }
};
class FakeModule : public yarp::os::RFModule, public googleSpeechProcess_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    FakeProcessing             *processing;
    friend class                processing;

    std::unordered_map<std::string,std::vector<std::string>> key_map;

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
        yarp::os::Bottle &general = rf.findGroup("general");
        if (general.isNull())
        {
            yError()<<"Unable to find \"general\"";
            return false;
        }
        std::string moduleName = general.check("name", yarp::os::Value("yarp-google-speech-process"), "module name (string)").asString();
        int numKey = general.find("num-keywords").asInt32();
        for (int i=0; i<numKey; i++)
        {
            std::string keywi = "keyword-"+std::to_string(i);
            yarp::os::Bottle &bKeyword = rf.findGroup(keywi);
            if (bKeyword.isNull())
            {
                yError()<<"Unable to find"<<keywi;
                return false;
            }
            if (!bKeyword.check("key") || !bKeyword.check("value"))
            {
                yError()<<"Unable to find \"key\" and/or \"value\"";
                return false;
            }
            std::string key = bKeyword.find("key").asString();
            yarp::os::Bottle *bValue = bKeyword.find("value").asList();
            std::vector<std::string> value(bValue->size());
            for (int j=0; j<bValue->size(); j++)
            {
                value[j] = bValue->get(j).asString();
            }

            key_map[key] = value;
        }

        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        processing = new FakeProcessing( moduleName, key_map );

        /* now start the thread to do the work */
        processing->open();

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
        processing->start_acquisition();
        return true;
    }

    /**********************************************************/
    bool stop()
    {
        processing->stop_acquisition();
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
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

    rf.setDefaultContext( "googleSpeechProcess" );
    rf.setDefaultConfigFile( "config.ini" );
    rf.setDefault("name","googleSpeechProcess");
    rf.configure(argc,argv);

    FakeModule fakemodule;
    return fakemodule.runModule(rf);
}
