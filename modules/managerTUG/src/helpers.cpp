#include "helpers.h"
#include <yarp/os/Time.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

#include <string>

using namespace std;
using namespace yarp::os;

namespace
{
    YARP_LOG_COMPONENT(HELPERS, "helpers")
}



bool reply(const string &s, const bool &wait,
           BufferedPort<Bottle> &speechPort,
           const RpcClient &speechRpcPort=yarp::os::RpcClient())
{
    bool ret=false;
    Bottle &b=speechPort.prepare();
    b.clear();
    b.addString(s);
    speechPort.writeStrict();
    while (wait && (speechPort.getOutputCount()>0))
    {
        Time::delay(0.01);
        Bottle cmd,rep;
        cmd.addVocab32("stat");
        if (speechRpcPort.write(cmd,rep))
        {
            if (rep.get(0).asString()=="quiet")
            {
                ret=true;
                break;
            }
        }
    }
    Time::delay(0.1);

    return ret;
}

bool checkOutputPorts(yarp::os::BufferedPort<yarp::os::Bottle>& port)
{
    if(port.getOutputCount() == 0)
    {
        yCDebug(HELPERS) << "Port" << port.getName() << "not connected.";
        return true;
    }
    return false;
}

bool checkInputPorts(yarp::os::BufferedPort<yarp::os::Bottle>& port)
{
    if(port.getInputCount() == 0)
    {
        yCDebug(HELPERS) << "Port" << port.getName() << "not connected.";
        return true;
    }
    return false;
}

bool checkPorts(yarp::os::RpcClient& port)
{
    if(port.getOutputCount() == 0)
    {
        yCDebug(HELPERS) << "Port" << port.getName() << "not connected.";
        return true;
    }
    return false;
}
