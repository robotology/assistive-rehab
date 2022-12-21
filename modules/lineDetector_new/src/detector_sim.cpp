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
bool Detector::get_model_pos(const std::string &model_name, yarp::sig::Vector &v)
{
    Bottle cmd,rep;
    cmd.addString("getModelPos");
    cmd.addString(model_name);
    if (m_gazeboPort.write(cmd,rep))
    {
        // Property prop(rep.get(0).toString().c_str());
        // Bottle *model=prop.find(model_name).asList();
        Bottle *model=rep.get(0).asList();
        if (Bottle *mod_bottle=model->find("pose_world").asList())
        {
            if(mod_bottle->size()>=7)
            {
                v.resize(7);
                v[0]=mod_bottle->get(0).asFloat64();
                v[1]=mod_bottle->get(1).asFloat64();
                v[2]=mod_bottle->get(2).asFloat64();
                v[3]=mod_bottle->get(3).asFloat64();
                v[4]=mod_bottle->get(4).asFloat64();
                v[5]=mod_bottle->get(5).asFloat64();
                v[6]=mod_bottle->get(6).asFloat64();
                return true;
            }
        }
    }
    return false;
}

/****************************************************************/
bool Detector::update_line_sim(const std::string &line_str)
{
    int idx= m_line2idx[line_str];
    return get_model_pos(line_str, m_lines_pose_world[idx]);
}
