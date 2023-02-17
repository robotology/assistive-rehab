#include "TriggerManager.h"
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>
#include "helpers.h"

namespace
{
    YARP_LOG_COMPONENT(TRIGGERMANAGER, "TriggerManager")
}

using namespace std;
using namespace yarp::os;


TriggerManager::TriggerManager(ResourceFinder &rf_, const bool &simulation_, const string &sentence_) : 
    PeriodicThread(0.5),
    rf(rf_), simulation(simulation_), sentence(sentence_), first_trigger(true),
    last_start_trigger(true), got_trigger(false), frozen(false), t0(Time::now()),
    t(Time::now())
{
    min_timeout=rf.check("min-timeout",Value(1.0)).asFloat64();
    max_timeout=rf.check("max-timeout",Value(10.0)).asFloat64();
}


TriggerManager::~TriggerManager()
{
}


void TriggerManager::setPorts(RpcClient *triggerPort, BufferedPort<Bottle> *speechPort, RpcClient *gazeboPort)
{
    this->triggerPort=triggerPort;
    this->speechPort=speechPort;
    this->gazeboPort=gazeboPort;
}

void TriggerManager::run()
{
    lock_guard<mutex> lg(mtx);
    double dt_from_last=t-t0;
    double time_elapsed=Time::now()-t;

    //if we received a trigger
    if (got_trigger)
    {
        got_trigger=false;
        if (first_trigger)
        {
            //if it's the first trigger, we assume it's a start
            first_trigger=false;
            if (simulation)
            {
                pause_actor();
                reply(sentence,false,*speechPort);
            }
            else
            {
                trigger_speech("start");
            }
            frozen=true;
            t0=t;
            return;
        }

        //if trigger occurs after the minimum timeout
        if (dt_from_last>=min_timeout)
        {
            //if last was a start trigger
            if (last_start_trigger)
            {
                //we send a stop
                last_start_trigger=false;
                if (!simulation)
                {
                    trigger_speech("stop");
                }
                frozen=false;
            }
            else //if last was a stop trigger
            {
                //we send a start
                last_start_trigger=true;
                if (simulation)
                {
                    pause_actor();
                    reply(sentence,false,*speechPort);
                }
                else
                {
                    trigger_speech("start");
                }
                frozen=true;
            }
            t0=t;
        }
        else
        {
            yCInfo(TRIGGERMANAGER)<<"Trigger occurred before min timeout";
            yCInfo(TRIGGERMANAGER)<<"Discarding trigger";
        }
        return;
    }

    //if we haven't received a trigger in the last max_timeout seconds
    //and last trigger received was a start
    if (last_start_trigger)
    {
        //we send a stop
        if (time_elapsed>max_timeout)
        {
            last_start_trigger=false;
            yCInfo(TRIGGERMANAGER)<<"Exceeded max timeout";
            if (!simulation)
            {
                trigger_speech("stop");
            }
            frozen=false;
            t0=t;
        }
    }
}


bool TriggerManager::freeze() const
{
    return frozen;
}


bool TriggerManager::restore() const
{
    return !frozen;
}


bool TriggerManager::pause_actor()
{
    Bottle cmd,rep;
    cmd.addString("pause");
    if (gazeboPort->write(cmd,rep))
    {
        if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
        {
            yCInfo(TRIGGERMANAGER)<<"Actor paused";
            return true;
        }
    }
    return false;
}


bool TriggerManager::trigger_speech(const string &s)
{
    yarp::os::Bottle cmd,rep;
    cmd.addString(s);
    if (triggerPort->write(cmd,rep))
    {
        if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
        {
            yCInfo(TRIGGERMANAGER)<<"Sending"<<s<<"to speech";
            if (s=="start")
            {
                reply(sentence,false,*speechPort);
            }
            return true;
        }
    }
    return false;
}


void TriggerManager::trigger()
{
    lock_guard<mutex> lg(mtx);
    t=Time::now();
    got_trigger=true;
    if (first_trigger)
    {
        t0=t;
        last_start_trigger=true;
    }
    yCInfo(TRIGGERMANAGER)<<"Got button trigger from the previous after"<<t-t0<<"seconds";
}


