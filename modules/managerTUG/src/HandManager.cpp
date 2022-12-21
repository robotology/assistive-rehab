class HandManager : public Thread
{
    string module_name;
    BufferedPort<Bottle> *opcPort;
    RpcClient *triggerPort;
    Skeleton *skeleton;
    double arm_thresh;
    string tag,part;
    bool isArmLifted;

public:

    /********************************************************/
    HandManager(const string &module_name_, const double &arm_thresh_)
        : module_name(module_name_), arm_thresh(arm_thresh_), tag(""),
          part(""), skeleton(NULL), isArmLifted(false) { }

    /********************************************************/
    void set_tag(const string &tag)
    {
        this->tag=tag;
    }

    /********************************************************/
    void setPorts(BufferedPort<Bottle> *opcPort, RpcClient *triggerPort)
    {
        this->opcPort=opcPort;
        this->triggerPort=triggerPort;
    }

    /********************************************************/
    void run() override
    {
        while(!isStopping())
        {
            if (opcPort->getInputCount() > 0)
            {
                if (!tag.empty())
                {
                    get_skeleton();
                    if (isArmLifted==false)
                    {
                        if (is_with_raised_hand())
                        {
                            yarp::os::Bottle cmd,rep;
                            cmd.addString("start");
                            isArmLifted=true;
                            triggerPort->write(cmd,rep);
                            if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
                            {
                                yInfo()<<"Starting speech";
                            }
                        }
                    }
                    else
                    {
                        if (!is_raised())
                        {
                            yarp::os::Bottle cmd,rep;
                            cmd.addString("stop");
                            isArmLifted=false;
                            triggerPort->write(cmd,rep);
                            if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
                            {
                                yInfo()<<"Stopping speech";
                            }
                        }
                    }
                }
            }
        }
    }

    /****************************************************************/
    void onStop() override
    {
        delete skeleton;
    }

    /****************************************************************/
    void get_skeleton()
    {
        if (Bottle* b=opcPort->read(true))
        {
            if (!b->get(1).isString())
            {
                for (int i=1; i<b->size(); i++)
                {
                    Property prop;
                    prop.fromString(b->get(i).asList()->toString());
                    string skel_tag=prop.find("tag").asString();
                    if (skel_tag==tag)
                    {
                        skeleton=skeleton_factory(prop);
                        skeleton->normalize();
                    }
                }
            }
        }
    }

    /****************************************************************/
    bool is_with_raised_hand()
    {
        if (is_raised("left"))
        {
            part="left";
            return true;
        }
        if (is_raised("right"))
        {
            part="right";
            return true;
        }

        return false;
    }

    /****************************************************************/
    bool is_raised() const
    {
        return is_raised(part);
    }

    /****************************************************************/
    bool is_raised(const string &p) const
    {
        string elbow=(p=="left"?KeyPointTag::elbow_left:KeyPointTag::elbow_right);
        string hand=(p=="left"?KeyPointTag::hand_left:KeyPointTag::hand_right);
        if (!p.empty())
        {
            if ((*skeleton)[elbow]->isUpdated() &&
                (*skeleton)[hand]->isUpdated())
            {
                if (((*skeleton)[hand]->getPoint()[2]-(*skeleton)[elbow]->getPoint()[2])>arm_thresh)
                {
                    return true;
                }
            }
        }

        return false;
    }

};
