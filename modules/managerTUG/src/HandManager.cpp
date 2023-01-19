#include "HandManager.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>
#include "helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace assistive_rehab;

namespace
{
    YARP_LOG_COMPONENT(HANDMANAGER, "HandManager")
}

HandManager::HandManager(const string &module_name_, const double &arm_thresh_)
    : module_name(module_name_), arm_thresh(arm_thresh_), tag(""),
        part(""), skeleton(NULL), isArmLifted(false) { }


void HandManager::set_tag(const string &tag)
{
    this->tag=tag;
}


void HandManager::setPorts(BufferedPort<Bottle> *opcPort, RpcClient *triggerPort)
{
    this->opcPort=opcPort;
    this->triggerPort=triggerPort;
}


void HandManager::run()
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
                            yCInfo(HANDMANAGER)<<"Starting speech";
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
                            yCInfo(HANDMANAGER)<<"Stopping speech";
                        }
                    }
                }
            }
        }
    }
}


void HandManager::onStop()
{
    delete skeleton;
}


void HandManager::get_skeleton()
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


bool HandManager::is_with_raised_hand()
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


bool HandManager::is_raised() const
{
    return is_raised(part);
}


bool HandManager::is_raised(const string &p) const
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

