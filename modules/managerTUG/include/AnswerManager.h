#pragma once

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <unordered_map>

class AnswerManager: public yarp::os::BufferedPort<yarp::os::Bottle>
{
    std::string module_name;
    std::unordered_map<std::string,std::string> speak_map;
    bool simulation;
    yarp::os::BufferedPort<yarp::os::Bottle> *speechPort;
    yarp::os::RpcClient *speechRpc;
    yarp::os::RpcClient *gazeboPort;
    bool replied;
    bool silent;
    double time;
    double speed;
    double speed_medium;
    double speed_high;
    std::mutex mtx;

public:
    AnswerManager(const std::string &module_name, 
                  const std::unordered_map<std::string,std::string> &speak_map,
                  const bool &simulation);

    ~AnswerManager();

    void setPorts(yarp::os::BufferedPort<yarp::os::Bottle> *speechPort, 
                  yarp::os::RpcClient *speechRpc,
                  yarp::os::RpcClient *gazeboPort);
                  
    void setExerciseParams(const double &distance, 
                           const double &time_high, 
                           const double &time_medium);

    bool open();

    bool connected();

    void close();

    void interrupt();

    void onRead( yarp::os::Bottle &answer );

    std::string get_actor_state();

    bool unpause_actor();

    void wakeUp();

    void suspend();

    void setSpeed(const double &speed);

    std::string getAnswer(const yarp::os::Bottle &answer);

    bool hasReplied();

    void reset();
};
