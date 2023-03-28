/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file collector.cpp
 * @authors: Valentina Vasco <valentina.vasco@iit.it> Alexandre Antunes <alexandre.gomespereira@iit.it>
 */

/**
 * Instructions of use:
 * This module has the following ports:
 *  - /eventCollector/speech:i               - this port receives strings from googleSpeech module
 *  - /eventCollector/speech-processing:i    - this port receives strings from googleSpeechProcessing module
 *  - /eventCollector/obstacle:i             - this port receives strings from obstacle(?) module
 *  - /eventCollector/skeleton:i             - this port receives strings from skeleton(?) module
 *  - /eventCollector/cmd                    - this port controls the flow of the module
 * 
 * To use this module, you must first ensure all ports are connected. 
 * At the beginning of each trial, you should send a "start" command through the /cmd port. This will tell the module
 * to listen to the ports and store the data.
 * At the end of a trial, you should send the "stop" command to the /cmd port to tell the module this trial is over.
 * The JSON file with all the collected data will be created when the module is closed.
 */

#include <iostream>
#include <vector>
#include <list>
#include <iomanip>
#include <mutex>
#include <fstream>

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>

#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

#include "src/eventCollector_IDL.h"

// #include "matio.h"

using namespace std;
using namespace yarp::os;

/****************************************************************/
class Collector : public RFModule, public eventCollector_IDL
{
    //params
    RpcServer rpcPort;
    BufferedPort<Bottle> googleSpeechPort;
    BufferedPort<Bottle> googleProcessPort;
    BufferedPort<Bottle> skeletonPort;
    BufferedPort<Bottle> obstacleDetectorPort;
    //vector<string> speech_events;
    //vector<string> speech_process_events;

    Json::Value jsonRoot;
    Json::Value jsonTrialInstance;
    Json::Value jsonNavErrorMessage;
    Json::Value jsonSkeletonErrorMessage;
    Json::Value jsonSpeechErrorMessage;
    bool started;
    bool got_speech;
    mutex mtx;

    int trialNumber;
    double timeFromStart;
    std::string skeletonTag;

    std::string configFileName;
    std::string outputFileName;
    std::string outfolder;

public:

    /********************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string moduleName = rf.check("name", Value("eventCollector")).asString();
        setName(moduleName.c_str());

        configFileName = rf.findFileByName("config.json");

        googleSpeechPort.open(("/" + getName() + "/speech:i").c_str());
        googleProcessPort.open(("/" + getName() + "/speech-process:i").c_str()); 
        obstacleDetectorPort.open(("/" + getName() + "/obstacle:i").c_str());
        skeletonPort.open(("/" + getName() + "/skeleton:i").c_str());

        rpcPort.open(("/" + getName() + "/cmd").c_str());
        attach(rpcPort);

        started=false;
        got_speech=false;
        
        jsonRoot["Trial"] = {}; 

        trialNumber = 0;
        outfolder = rf.getHomeContextPath();

        return true;
    }

    /********************************************************/
    bool interruptModule()
    {
        googleSpeechPort.interrupt();
        googleProcessPort.interrupt();
        obstacleDetectorPort.interrupt();
        skeletonPort.interrupt();
        rpcPort.interrupt();
        yInfo() << "Interrupted module";
        return true;
    }

    /********************************************************/
    bool close()
    {
        if (started)
        {
             stop();
        }
        googleSpeechPort.close();
        googleProcessPort.close();
        obstacleDetectorPort.close();
        skeletonPort.close();
        rpcPort.close();
        yInfo() << "Closed ports";
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
        lock_guard<mutex> lg(mtx);

        if (!started)
        {
            return true;
        }

        Bottle *speech=googleSpeechPort.read(false);
        Bottle *speech_process=googleProcessPort.read(false);
        Bottle *obstacle = obstacleDetectorPort.read(false);
        Bottle *skeleton = skeletonPort.read(false);

        if (speech)
        {
            if (speech->size()>0)
            {
                if (got_speech) // this means we got a speech bottle after another speechBottle, something went wrong
                {
                    // in this situation, we save first the previous instance

                    jsonSpeechErrorMessage["google-speech-process"]="no google-speech-processing triggered";
                    jsonTrialInstance["Speech"]["error-messages"].append(jsonSpeechErrorMessage);
                }
                yDebug()<<"speech bottle:"<<speech->toString();
                string content=speech->get(0).asString();
                yDebug()<<"content:"<<content;

                jsonSpeechErrorMessage["google-speech-event-time"] = yarp::os::Time::now() - timeFromStart;

                jsonSpeechErrorMessage["google-speech"]=content;

                got_speech=true;
                
            }
            // we save the time we got a speech bottle, in case we get two in a row
            jsonSpeechErrorMessage["google-speech-process-event-time"] = yarp::os::Time::now() - timeFromStart;
        }

        if (speech_process) // if we got something from speech process AND we already had something from speech
        {
            if (speech_process->size()>0)
            {
                if (!got_speech) // similarly, in this case the speechProcessing was triggered by something else
                {
                    // in this case we just append an error mesage to google-speech before processing
                    jsonSpeechErrorMessage["google-speech-event-time"] = yarp::os::Time::now() - timeFromStart;
                    jsonSpeechErrorMessage["google-speech"]="no google-speech triggered";
                }
                yDebug()<<"speech process bottle:"<<speech_process->toString();
                string content=speech_process->get(0).asString();
                yDebug()<<"content:"<<content;

                jsonSpeechErrorMessage["google-speech-process-event-time"] = yarp::os::Time::now() - timeFromStart;
                jsonSpeechErrorMessage["google-speech-process"]=content;
                jsonTrialInstance["Speech"]["error-messages"].append(jsonSpeechErrorMessage);

                got_speech=false;
            }
        }

        if(obstacle)
        {
            if(obstacle->size()>0)
            {
                yDebug()<<"obstacle detector bottle:" << obstacle->toString();
                string content="found obstacle from " + obstacle->get(1).asString() + 
                                " at distance " + std::to_string(obstacle->get(0).asFloat64());
                yDebug()<<"content:"<<content;

                jsonNavErrorMessage["obstacle-detection-event-time"] = yarp::os::Time::now() - timeFromStart;
                jsonNavErrorMessage["obstacle-detection"] = content;
                jsonTrialInstance["Navigation"]["error-messages"].append(jsonNavErrorMessage);
            }
        }
 
        if(skeleton)
        {
            if(skeleton->size()>0)
            {
                yDebug()<<"skeleton bottle:" << skeleton->toString();
                string content="skeleton-misdetection:" + skeleton->get(0).asString();
                yDebug()<<"content:"<<content;

                jsonSkeletonErrorMessage["skeleton-misdetection-event-time"] = yarp::os::Time::now() - timeFromStart;
                jsonSkeletonErrorMessage["skeleton-misdetection"] = content;
                jsonTrialInstance["skeleton"]["error-messages"].append(jsonSkeletonErrorMessage);
            }
        }

        return true;
    }

    /****************************************************************/
    bool save_data()
    {
        lock_guard<mutex> lg(mtx);

        auto time = std::time(nullptr);
        auto tm = *std::localtime(&time);
        std::ostringstream date_stream;
        date_stream << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
        std::ostringstream date_stream_compact;
        date_stream_compact << std::put_time(&tm, "%d%m%Y-%H%M%S");
        
        jsonTrialInstance["Stop-Time"] = date_stream.str();

        // we had the trial instance to the trial list when we stop a trial
        jsonRoot["Trial"].append(jsonTrialInstance);

#if 0
        // set the output filename as the skeletonTag. Beware: file will be overwritten by a new skeleton with the same id!
        outputFileName = outfolder + "/collectedEvents_user" + skeletonTag + ".json";
#else
        // set the output filename as the date+skeletonTag. No overwrites are possibile
        outputFileName = outfolder + "/collectedEvents_date" + date_stream_compact.str() + "_user" + skeletonTag + ".json";
#endif

        yInfo() << "Saving events in" << outputFileName;
        std::ofstream output_doc(outputFileName.c_str(), std::ofstream::binary);
        output_doc << jsonRoot;

        jsonRoot["Trial"] = {}; 

        return true;

        // {
        //     "Trial" : 
        //     [         
                    // {
                    //     "Number": 0,
                    //     "Speech" : 
                    //     {
                    //             "error-messages" : [ 
                    //                 { "google-speech": "e1",
                    //                 "google-speech-process": "e1process"
                    //                 },
                    //                 { "google-speech": "e2",
                    //                 "google-speech-process": "e2process"
                    //                 }
                    //             ]
                    //     },
                    //     "Navigation" : 
                    //     {
                    //             "error-messages" : [ "e1", "e2" ]
                    //     }
                    // }
        //     ]
        // }


    }

    /****************************************************************/
    //bool save()
    //{
        // //save to out file
        // const char *filename = "myfile.mat";
        // mat_t *matfp = NULL; //matfp contains pointer to MAT file or NULL on failure
        // matfp = Mat_CreateVer(filename, NULL, MAT_FT_MAT5); //or MAT_FT_MAT4 / MAT_FT_MAT73
        // //don't forget to close file with Mat_Close(matfp);


        // char* fieldname = "MyStringVariable";
        // char* mystring = "Text";
        // char* mystringnew[] = { mystring, mystring };
        // char foo[3][3] = {{'a','b','c'}, {'d','e','f'},{'g','h','i'}};
        // yDebug()<<__LINE__<<strlen(foo[1]);
        // size_t dim[2] = { 2, 4 };
        // matvar_t *variable = Mat_VarCreate(fieldname, MAT_C_CHAR, MAT_T_UTF8, 2, dim, &mystringnew, 0);
        // Mat_VarWrite(matfp, variable, MAT_COMPRESSION_NONE); //or MAT_COMPRESSION_ZLIB
        // Mat_VarFree(variable);

        // const int first = 3; //rows
        // string array1d[first]= { "ciaoo", "hello", "brabr" };

        // char* fieldname1d = "array1d";
        // size_t dim1d[2] = { first, 5 };
        // matvar_t *variable1d = Mat_VarCreate(fieldname1d, MAT_C_CHAR, MAT_T_UTF8, 2, dim1d, &array1d, 0); //rank 1
        // Mat_VarWrite(matfp, variable1d, MAT_COMPRESSION_NONE);
        // Mat_VarFree(variable1d);

        // // fill 1d array
        // for (int i = 0; i < first; i++)
        //     array1d[i] = (i + 1);

        // // write
        // char* fieldname1d = "speech_process";
        // size_t dim1d[2] = { speech_process_events.size(), 50 };
        // matvar_t *variable1d = Mat_VarCreate(fieldname1d, MAT_C_CHAR, MAT_T_UTF8, 2, dim1d, &speech_process_events, 0); //rank 1
        // Mat_VarWrite(matfp, variable1d, MAT_COMPRESSION_NONE);
        // Mat_VarFree(variable1d);

    //    return true;
    //}

    /****************************************************************/
    bool start(const string &skeleton_tag) override
    {
        lock_guard<mutex> lg(mtx);
        yInfo()<<"Starting collecting";

        // Read file - this should probable be on the config step
        std::ifstream config_doc(configFileName.c_str(), std::ifstream::binary);

        // when we start, we prepare an instance of a trial
        jsonTrialInstance["Instance"] = {}; 
        // and populate it with the structure
        config_doc >> jsonTrialInstance; 
        // we add the trial number to the instance
        jsonTrialInstance["Number"] = trialNumber;
        jsonTrialInstance["User-id"] = skeleton_tag;
        skeletonTag = skeleton_tag;

        // now we get the current date in order to store it on the instance
        auto time = std::time(nullptr);
        auto tm = *std::localtime(&time);
        std::ostringstream date_stream;
        date_stream << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
        jsonTrialInstance["Start-Time"] = date_stream.str();

        timeFromStart = yarp::os::Time::now();

        trialNumber++;

        // We start measuring the time to monitor events

        started = true;
        return true;
    }

    /****************************************************************/
    bool stop() override
    {   
        save_data();
        started=false;
        return true;
    }

};


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("eventCollector");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    Collector collector;
    return collector.runModule(rf);
}

