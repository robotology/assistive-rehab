#pragma once

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <mutex>
#include <string>


class TriggerManager: public yarp::os::PeriodicThread
{
private:
    yarp::os::ResourceFinder rf;
    std::string module_name;
    double min_timeout;
    double max_timeout;
    double t0,t;
    bool simulation;
    yarp::os::RpcClient *triggerPort;
    yarp::os::BufferedPort<yarp::os::Bottle> *speechPort;
    yarp::os::RpcClient *gazeboPort;
    bool first_trigger;
    bool last_start_trigger;
    bool got_trigger,frozen;
    std::string sentence;
    std::mutex mtx;

public:


    TriggerManager(yarp::os::ResourceFinder &rf_, const bool &simulation_, const std::string &sentence_);


    ~TriggerManager();
    
    void setPorts(yarp::os::RpcClient *triggerPort, 
                  yarp::os::BufferedPort<yarp::os::Bottle> *speechPort, 
                  yarp::os::RpcClient *gazeboPort);


    void run() override;


    bool freeze() const;


    bool restore() const;


    bool pause_actor();


    bool trigger_speech(const std::string &s);
    

    void trigger();
    
};
