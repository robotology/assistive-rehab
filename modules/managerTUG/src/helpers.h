#include <string>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>



bool reply(const std::string &s, const bool &wait,
           yarp::os::BufferedPort<yarp::os::Bottle> &speechPort,
           const yarp::os::RpcClient &speechRpcPort=yarp::os::RpcClient());

bool checkOutputPorts(yarp::os::BufferedPort<yarp::os::Bottle>& port);

bool checkInputPorts(yarp::os::BufferedPort<yarp::os::Bottle>& port);

bool checkPorts(yarp::os::RpcClient& port);


