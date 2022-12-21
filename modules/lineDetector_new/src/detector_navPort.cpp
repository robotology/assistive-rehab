#include <iostream>
#include <mutex>
#include <condition_variable>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/cv/Cv.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/math/Math.h>

#include "detector.h"
#include "src/lineDetector_IDL.h"

using namespace yarp::os;
using namespace yarp::math;

/****************************************************************/
bool Detector::update_nav_frame()
{
    Bottle cmd,rep;
    cmd.addString("get_state");
    if (m_navPort.write(cmd,rep))
    {
        Bottle *robotState=rep.get(0).asList();
        if (Bottle *loc=robotState->find("robot-location").asList())
        {
            yarp::sig::Vector robot_location(7,0.0);
            robot_location[0]=loc->get(0).asFloat64();
            robot_location[1]=loc->get(1).asFloat64();
            robot_location[5]=1.0;
            robot_location[6]=(M_PI/180)*loc->get(2).asFloat64();
            m_navFrame=yarp::math::axis2dcm(robot_location.subVector(3,6));
            m_navFrame.setSubcol(robot_location.subVector(0,2),0,3);
            m_updated_nav=true;
            return true;
        }
    }
    return false;
}

/****************************************************************/
bool Detector::update_odometry(const double x, const double y, const double theta)
{
    Bottle cmd,rep;
    cmd.addString("reset_odometry");
    cmd.addFloat64(x);
    cmd.addFloat64(y);
    cmd.addFloat64(theta);
    if (m_navPort.write(cmd,rep))
    {
        if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
        {
            m_updated_odom=true;
            yInfo()<<"Reset robot's odometry";
            return true;
        }
    }
    return false;
}

/****************************************************************/
bool Detector::update_odometry(const std::string &line_tag, const double theta)
{
    std::lock_guard<std::mutex> lg(m_mtx_update);
    if (m_line2idx.count(line_tag)==0)
    {
        return false;
    }
    int i= m_line2idx[line_tag];
    yarp::sig::Vector pose= m_lines_pose_world[i].subVector(0,2);
    return update_odometry(pose[0],pose[1],theta);
}

/****************************************************************/
bool Detector::go_to_line(const std::string &line_tag, const double theta)
{
    std::lock_guard<std::mutex> lg(m_mtx_update);
    if (m_updated_odom && m_line2idx.count(line_tag)>0)
    {
        yInfo()<<"Going to"<<line_tag;
        int i= m_line2idx[line_tag];
        yarp::sig::Vector pose= m_lines_pose_world[i].subVector(0,2);
        Bottle cmd,rep;
        cmd.addString("go_to_wait");
        cmd.addFloat64(pose[0]);
        cmd.addFloat64(pose[1]);
        cmd.addFloat64(theta);
        if (m_navPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
            {
                yInfo()<<"Reached"<<line_tag;
                cmd.clear();
                rep.clear();
                cmd.addString("reset_odometry");
                cmd.addFloat64(pose[0]);
                cmd.addFloat64(pose[1]);
                cmd.addFloat64(theta);
                if (m_navPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
                    {
                        yInfo()<<"Reset robot's odometry";
                        return true;
                    }
                }
            }
        }
    }
    else
    {
        yWarning()<<"The line does not exist or odometry is not referred to world frame!";
    }

    return false;
}
