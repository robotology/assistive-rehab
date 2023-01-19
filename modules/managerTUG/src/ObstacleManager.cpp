#include "ObstacleManager.h"
#include <yarp/os/Time.h>

using namespace std;
using namespace yarp::os;


namespace
{
    YARP_LOG_COMPONENT(OBSTACLEMANAGER, "ObstacleManager")
}

ObstacleManager::ObstacleManager(const string & module_name, BufferedPort<Bottle> *obstaclePort)
    : PeriodicThread(0.1)
{
    this->module_name=module_name;
    this->obstaclePort=obstaclePort;
    silent=true;
    found=false;
    first=true;
    tlast=Time::now();
}


ObstacleManager::~ObstacleManager()
{
}


void ObstacleManager::run()
{
    if (!silent)
    {
        lock_guard<mutex> lg(mtx);
        if (Bottle *input=obstaclePort->read(false))
        {
            // we check the frequency of occurrence is small
            // otherwise it might be a false positive
            if ((Time::now()-tlast)<0.2)
            {
                found=true;
                laser=input->get(1).asString();
                tlast=Time::now();
            }

            if (first)
            {
                tlast=Time::now();
                first=false;
            }
        }
        else
        {
            // if we don't detect obstacle for at least 0.5 seconds
            if ((Time::now()-tlast)>0.5)
            {
                found=false;
            }
        }
    }
}


string ObstacleManager::whichLaser() const
{
    return laser;
}


bool ObstacleManager::hasObstacle() const
{
    return found;
}   


void ObstacleManager::wakeUp()
{
    silent=false;
}


void ObstacleManager::suspend()
{
    silent=true;
    found=false;
    first=true;
}    

