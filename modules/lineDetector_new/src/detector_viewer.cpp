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
bool Detector::create_line_viewer(const std::string &line_tag)
{
    int i= m_line2idx[line_tag];
    double llen=(m_marker_dist[i]+ m_marker_size[i])* m_nx[i];
    yarp::sig::Vector ax(4);
    ax[0]= m_lines_pose_world[i][3];
    ax[1]= m_lines_pose_world[i][4];
    ax[2]= m_lines_pose_world[i][5];
    ax[3]= m_lines_pose_world[i][6];
    yarp::sig::Matrix R=axis2dcm(ax);
    yarp::sig::Vector u=R.subcol(0,0,2);
    yarp::sig::Vector p0(3);
    p0[0]= m_lines_pose_world[i][0];
    p0[1]= m_lines_pose_world[i][1];
    p0[2]= m_lines_pose_world[i][2];
    yarp::sig::Vector p1=p0+llen*u;
    std::vector<int> color(3,0);
    if(line_tag=="start-line")
    {
        color[0]=1;
    }
    if(line_tag=="finish-line")
    {
        color[2]=1;
    }
    Bottle cmd,rep;
    cmd.addString("create_line");
    cmd.addString(line_tag);
    cmd.addFloat64(p0[0]);
    cmd.addFloat64(p0[1]);
    cmd.addFloat64(p0[2]);
    cmd.addFloat64(p1[0]);
    cmd.addFloat64(p1[1]);
    cmd.addFloat64(p1[2]);
    cmd.addInt32(color[0]);
    cmd.addInt32(color[1]);
    cmd.addInt32(color[2]);
    if(m_viewerPort.write(cmd,rep))
    {
        if(rep.get(0).asBool()==true)
        {
            yInfo()<<line_tag<<"created on the viewer";
            return true;
        }
    }
    return false;
}

/****************************************************************/
void Detector::delete_line_viewer(const std::string &l)
{
    Bottle cmd,rep;
    cmd.addString("delete_line");
    cmd.addString(l);
    if(m_viewerPort.write(cmd,rep))
    {
        if(rep.get(0).asVocab32()==Vocab32::encode("ok"))
        {
            yInfo()<<l<<"deleted on the viewer";
            m_updated_line_viewer=false;
        }
    }
}

