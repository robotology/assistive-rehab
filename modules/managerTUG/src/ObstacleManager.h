#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/LogComponent.h>
#include "helpers.h"

#include <mutex>



class ObstacleManager : public yarp::os::PeriodicThread
{
    yarp::os::BufferedPort<yarp::os::Bottle> *obstaclePort;
    std::string module_name;
    std::mutex mtx;
    double tlast;
    bool silent;
    bool found;
    bool first;
    std::string laser;

public:

    ObstacleManager(const std::string & module_name, yarp::os::BufferedPort<yarp::os::Bottle> *obstaclePort);

    ~ObstacleManager();
    
    void run() override;

    std::string whichLaser() const;

    bool hasObstacle() const;

    void wakeUp();

    void suspend();
};
