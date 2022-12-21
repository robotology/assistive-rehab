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

using namespace yarp::os;
using namespace yarp::math;

/****************************************************************/
void Detector::create_line_tf(const std::string &line_tag)
{
    if (m_itf != nullptr)
    {
        int i = m_line2idx[line_tag];
        yarp::sig::Vector ax(4);
        ax[0] = m_lines_pose_world[i][3];
        ax[1] = m_lines_pose_world[i][4];
        ax[2] = m_lines_pose_world[i][5];
        ax[3] = m_lines_pose_world[i][6];
        yarp::sig::Matrix R = axis2dcm(ax);
        R[0][3] = m_lines_pose_world[i][0];
        R[1][3] = m_lines_pose_world[i][1];
        R[2][3] = m_lines_pose_world[i][2];
        if (line_tag == "start-line") { m_itf->setTransform("odom" , "start-line-frame",R); }
        else if (line_tag == "end-line") { m_itf->setTransform("odom", "end-line-frame", R); }
    }
}

/****************************************************************/
void Detector::delete_line_tf(const std::string& line_tag)
{
    if (m_itf != nullptr)
    {
        if (line_tag == "start-line") { m_itf->deleteTransform("odom", "start-line-frame"); }
        else if (line_tag == "end-line") { m_itf->deleteTransform("odom", "end-line-frame"); }
    }
}
