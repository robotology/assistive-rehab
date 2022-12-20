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
bool Detector::update_cam_frame()
{
    if (Property *p= m_gazeStatePort.read(false))
    {
        yarp::sig::Vector pose;
        p->find("depth_rgb").asList()->write(pose);
        m_camFrame=yarp::math::axis2dcm(pose.subVector(3,6));
        m_camFrame.setSubcol(pose.subVector(0,2),0,3);
        m_updated_cam=true;
        return true;
    }
    return false;
}

/****************************************************************/
bool Detector::getCameraOptions(cv::Mat &cam_intrinsics, cv::Mat &cam_distortion)
{
    yarp::dev::IRGBDSensor* irgbd;
    m_rgbdDrv.view(irgbd);

    if (irgbd != nullptr) {
        yInfo() << "Reading intrinsics from camera";
        yarp::os::Property intrinsics;
        if (irgbd->getRgbIntrinsicParam(intrinsics)) {
            m_fx = intrinsics.find("focalLengthX").asFloat64();
            m_fy = intrinsics.find("focalLengthY").asFloat64();
            m_px = intrinsics.find("principalPointX").asFloat64();
            m_py = intrinsics.find("principalPointY").asFloat64();
                
            yInfo() << "Camera intrinsics:" << m_fx << m_fy << m_px << m_py;
                
            cam_intrinsics.at<double>(0, 0) = m_fx;
            cam_intrinsics.at<double>(0, 2) = m_px;
            cam_intrinsics.at<double>(1, 1) = m_fy;
            cam_intrinsics.at<double>(1, 2) = m_py;
            cam_intrinsics.at<double>(2, 2) = 1.0;

            cam_distortion.at<double>(0, 0) = 0.0; //k1;
            cam_distortion.at<double>(0, 1) = 0.0; //k2;
            cam_distortion.at<double>(0, 2) = 0.0; //t1;
            cam_distortion.at<double>(0, 3) = 0.0; //t2;
            cam_distortion.at<double>(0, 4) = 0.0; //k3;

            m_rgbdDrv.close();
            return true;
        }
    }

    return false;
}




/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"Unable to find Yarp server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    Detector line_detector;
    return line_detector.runModule(rf);
}
