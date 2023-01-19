#include "AnswerManager.h"
#include <condition_variable>
#include "helpers.h"

#include <mutex>

#include <cmath>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/math/Math.h>
#include <yarp/os/LogComponent.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

namespace
{
    YARP_LOG_COMPONENT(ANSWERMANAGER, "AnswerManager")
}


AnswerManager::AnswerManager(const string &module_name_, const unordered_map<string,string> &speak_map_,
                const bool &simulation_) : module_name(module_name_), speak_map(speak_map_),
    simulation(simulation_), time(0.0), silent(true), replied(false) { }


AnswerManager::~AnswerManager()
{
}


void AnswerManager::setPorts(BufferedPort<Bottle> *speechPort, RpcClient *speechRpc,
                RpcClient *gazeboPort)
{
    yCDebug(ANSWERMANAGER) << "setPorts";
    this->speechPort=speechPort;
    this->speechRpc=speechRpc;
    this->gazeboPort=gazeboPort;
}


void AnswerManager::setExerciseParams(const double &distance, const double &time_high, const double &time_medium)
{
    yCDebug(ANSWERMANAGER) << "setExerciseParams";
    speed_high=(2.0*distance)/time_high;
    speed_medium=(2.0*distance)/time_medium;
}


bool AnswerManager::open()
{
    yCDebug(ANSWERMANAGER) << "open";
    this->useCallback();
    BufferedPort<yarp::os::Bottle >::open("/"+module_name+"/answer:i");
    return true;
}


bool AnswerManager::connected()
{
    yCDebugThrottle(ANSWERMANAGER, 5) << "answer_manager input count" << this->getInputCount();
    if(this->getInputCount() == 0)
    {
        yCDebugThrottle(ANSWERMANAGER, 5) << this->getName() << "not connected.";
        return false;
    }
    return true;
}


void AnswerManager::close()
{
    yCDebug(ANSWERMANAGER) << "close";
    BufferedPort<yarp::os::Bottle >::close();
}


void AnswerManager::interrupt()
{
    yCDebug(ANSWERMANAGER) << "interrupt";
    BufferedPort<yarp::os::Bottle >::interrupt();
}


void AnswerManager::onRead( yarp::os::Bottle &answer )
{
    yCDebug(ANSWERMANAGER) << "onRead answer is" << answer.toString().c_str();
    lock_guard<mutex> lg(mtx);
    if(!silent)
    {
        yInfo()<<"Trying to answer...";
        string keyword=getAnswer(answer);
        replied=reply(speak_map[keyword],true,*speechPort,*speechRpc);
        if (simulation)
        {
            string actor_state=get_actor_state();
            if (actor_state!="sit_down" && actor_state!="sitting")
            {
                unpause_actor();
            }
        }
    }
    else {
        yCDebug(ANSWERMANAGER) << "onRead is silent";
    }
}


string AnswerManager::get_actor_state()
{
    yCDebug(ANSWERMANAGER) << "get_actor_state";
    string state="";
    Bottle cmd,rep;
    cmd.addString("getState");
    if (gazeboPort->write(cmd,rep))
    {
        state=rep.get(0).asString();
    }
    return state;
}


bool AnswerManager::unpause_actor()
{
    yCDebug(ANSWERMANAGER) << "unpause_actor";
    Bottle cmd,rep;
    cmd.addString("playFromLast");
    cmd.addInt32(1);
    if (gazeboPort->write(cmd,rep))
    {
        if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
        {
            yInfo()<<"Actor unpaused";
            return true;
        }
    }
    return false;
}


void AnswerManager::wakeUp()
{
    yCDebug(ANSWERMANAGER) << "wakeUp";
    silent=false;
}


void AnswerManager::suspend()
{
    yCDebug(ANSWERMANAGER) << "suspend";
    silent=true;
}


void AnswerManager::setSpeed(const double &s)
{
    this->speed=s;
}


string AnswerManager::getAnswer(const Bottle &b)
{
    string keyword="";
    if(!b.isNull())
    {
        keyword=b.get(0).asString();
        if (!keyword.empty())
        {
            if(keyword=="feedback")
            {
                if(speed>=speed_high)
                {
                    keyword+="-high";
                }
                else if(speed<speed_high && speed>=speed_medium)
                {
                    keyword+="-medium";
                }
                else if(speed<speed_medium)
                {
                    keyword+="-low";
                }
            }
            yInfo()<<"Received keyword"<<keyword;
        }
        if (keyword.empty() || speak_map.count(keyword)<=0)
        {
            yInfo()<<"Could not understand the question";
            yInfo()<<"Asking to repeat the question";
            keyword="unclear";
        }
    }
    else
    {
        yWarning()<<"Corrupted bottle";
    }

    return keyword;
}


bool AnswerManager::hasReplied()
{
    lock_guard<mutex> lg(mtx);
    return replied;
}


void AnswerManager::reset()
{
    lock_guard<mutex> lg(mtx);
    replied=false;
}

