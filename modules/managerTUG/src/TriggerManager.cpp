class TriggerManager: public Thread
{
    ResourceFinder rf;
    string module_name;
    double min_timeout;
    double max_timeout;
    double t0,t;
    bool simulation;
    RpcClient *triggerPort;
    BufferedPort<Bottle> *speechPort;
    RpcClient *gazeboPort;
    bool first_trigger,last_start_trigger,got_trigger,freezing;
    string sentence;
    mutex mtx;

public:

    /********************************************************/
    TriggerManager(ResourceFinder &rf_, const bool &simulation_, const string &sentence_) :
        rf(rf_), simulation(simulation_), sentence(sentence_), first_trigger(true),
        last_start_trigger(true), got_trigger(false), freezing(false), t0(Time::now()),
        t(Time::now())
    {
        min_timeout=rf.check("min-timeout",Value(1.0)).asFloat64();
        max_timeout=rf.check("max-timeout",Value(10.0)).asFloat64();
    }

    /********************************************************/
    ~TriggerManager()
    {
    }

    /********************************************************/
    void setPorts(RpcClient *triggerPort, BufferedPort<Bottle> *speechPort, RpcClient *gazeboPort)
    {
        this->triggerPort=triggerPort;
        this->speechPort=speechPort;
        this->gazeboPort=gazeboPort;
    }

    /********************************************************/
    void run() override
    {
        while(!isStopping())
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
                    freezing=true;
                    t0=t;
                    continue;
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
                        freezing=false;
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
                        freezing=true;
                    }
                    t0=t;
                }
                else
                {
                    yInfo()<<"Trigger occurred before min timeout";
                    yInfo()<<"Discarding trigger";
                }
                continue;
            }

            //if we haven't received a trigger in the last max_timeout seconds
            //and last trigger received was a start
            if (last_start_trigger)
            {
                //we send a stop
                if (time_elapsed>max_timeout)
                {
                    last_start_trigger=false;
                    yInfo()<<"Exceeded max timeout";
                    if (!simulation)
                    {
                        trigger_speech("stop");
                    }
                    freezing=false;
                    t0=t;
                }
            }
        }
    }

    /********************************************************/
    bool freeze() const
    {
        return freezing;
    }

    /********************************************************/
    bool restore() const
    {
        return !freezing;
    }

    /********************************************************/
    bool pause_actor()
    {
        Bottle cmd,rep;
        cmd.addString("pause");
        if (gazeboPort->write(cmd,rep))
        {
            if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
            {
                yInfo()<<"Actor paused";
                return true;
            }
        }
        return false;
    }

    /********************************************************/
    bool trigger_speech(const string &s)
    {
        yarp::os::Bottle cmd,rep;
        cmd.addString(s);
        if (triggerPort->write(cmd,rep))
        {
            if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
            {
                yInfo()<<"Sending"<<s<<"to speech";
                if (s=="start")
                {
                    reply(sentence,false,*speechPort);
                }
                return true;
            }
        }
        return false;
    }

    /********************************************************/
    void trigger()
    {
        lock_guard<mutex> lg(mtx);
        t=Time::now();
        got_trigger=true;
        if (first_trigger)
        {
            t0=t;
            last_start_trigger=true;
        }
        yInfo()<<"Got button trigger from the previous after"<<t-t0<<"seconds";
    }

};
