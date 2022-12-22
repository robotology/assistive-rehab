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

#include <iCub/ctrl/filters.h>

#include <detector.h>

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"

using namespace yarp::os;
using namespace yarp::math;

/****************************************************************/
bool Detector::attach(RpcServer &source)
{
    return yarp().attachAsServer(source);
}

/****************************************************************/
bool Detector::configure(ResourceFinder &rf)
{
    yDebug() << rf.toString(); 
    //parameters
    std::string moduleName=rf.check("name", Value("lineDetector")).asString();
    setName(moduleName.c_str());

    m_period=rf.check("period",Value(0.1)).asFloat64();
    m_nlines=rf.check("nlines",Value(2)).asInt32();
    m_line_filter_order=rf.check("line-filter-order",Value(30)).asInt32();
    m_use_initial_guess = rf.check("use-initial-guess", Value(false)).asBool();

    m_nx={6,6};
    if(Bottle *nxB=rf.find("nx").asList())
    {
        if(nxB->size()!= m_nlines)
        {
            yError()<<"The number of parameters has to match the number of lines";
            yError()<<"nx="<<nxB->toString();
            return false;
        }
        m_nx[0]=nxB->get(0).asInt32();
        m_nx[1]=nxB->get(1).asInt32();
    }

    m_ny={1,1};
    if(Bottle *nyB=rf.find("ny").asList())
    {
        if(nyB->size()!= m_nlines)
        {
            yError()<<"The number of parameters has to match the number of lines";
            yError()<<"ny="<<nyB->toString();
            return false;
        }
        m_ny[0]=nyB->get(0).asInt32();
        m_ny[1]=nyB->get(1).asInt32();
    }

    m_marker_size={0.13,0.13};
    if(Bottle *mSzB=rf.find("marker-size").asList())
    {
        if(mSzB->size()!= m_nlines)
        {
            yError()<<"The number of parameters has to match the number of lines";
            yError()<<"marker-size="<<mSzB->toString();
            return false;
        }
        m_marker_size[0]=mSzB->get(0).asFloat64();
        m_marker_size[1]=mSzB->get(1).asFloat64();
    }

    m_marker_dist={0.005,0.005};
    if(Bottle *mDistB=rf.find("marker-dist").asList())
    {
        if(mDistB->size()!= m_nlines)
        {
            yError()<<"The number of parameters has to match the number of lines";
            yError()<<"marker-dist="<<mDistB->toString();
            return false;
        }
        m_marker_dist[0]=mDistB->get(0).asFloat64();
        m_marker_dist[1]=mDistB->get(1).asFloat64();
    }

    m_line2idx["start-line"]=0;
    m_line2idx["finish-line"]=1;

    //create dictionary
    //0 corresponds to the starting line
    //1 corresponds to the ending line
    m_dictionary.resize(m_nlines);
    m_dictionary[0]=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    m_dictionary[1]=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);

    m_board.resize(m_nlines);
    m_lines_pose_world.resize(m_nlines);
    m_lines_filter.resize(m_nlines);
    m_lines_size.resize(m_nlines);
    for(int i=0; i< m_nlines;i++)
    {
        m_board[i]=cv::aruco::GridBoard::create(m_nx[i], m_ny[i], m_marker_size[i], m_marker_dist[i], m_dictionary[i]);
        yDebug()<<"done";
        m_lines_pose_world[i]=yarp::sig::Vector(7,0.0);
        m_lines_filter[i]=new iCub::ctrl::MedianFilter(m_line_filter_order,yarp::sig::Vector(7,0.0));
        m_lines_size[i]=yarp::sig::Vector(2,0.0);
        m_lines_size[i][0]=(m_marker_dist[i]+ m_marker_size[i])* m_nx[i];
        m_lines_size[i][1]= m_marker_size[i]* m_ny[i];
        yDebug("33");
    }
yDebug("3");
    m_rvec_v.resize(m_nlines);
    for(int i=0; i< m_nlines;i++)
    {
        m_rvec_v[i] = cv::Vec3d(0.0, 0.0, 0.0);
    }      
    m_tvec_v.resize(m_nlines);

    for(int i=0; i< m_nlines;i++)
    {
        m_tvec_v[i] = cv::Vec3d(0.0, 0.0, 0.0);
    }

yDebug("2");
    //configure the detector with corner refinement
    m_detector_params.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    std::string camera_remote = "/depthCamera";
    m_cam_intrinsic=cv::Mat::eye(3,3,CV_64F);
    m_cam_distortion=cv::Mat::zeros(1,5,CV_64F);
    Bottle &gCamera=rf.findGroup("camera");
    if (!gCamera.isNull())
    {
        if (Bottle *intrinsics=gCamera.find("intrinsics").asList())
        {
            if (intrinsics->size()>=4)
            {
                m_camera_configured=true;
                m_fx=intrinsics->get(0).asFloat64();
                m_fy=intrinsics->get(1).asFloat64();
                m_px=intrinsics->get(2).asFloat64();
                m_py=intrinsics->get(3).asFloat64();
                yInfo()<<"camera fx (from file) ="<< m_fx;
                yInfo()<<"camera fy (from file) ="<< m_fy;
                yInfo()<<"camera px (from file) ="<< m_px;
                yInfo()<<"camera py (from file) ="<< m_py;

                m_cam_intrinsic.at<double>(0,0)= m_fx;
                m_cam_intrinsic.at<double>(0,2)= m_px;
                m_cam_intrinsic.at<double>(1,1)= m_fy;
                m_cam_intrinsic.at<double>(1,2)= m_py;

                m_cam_distortion.at<double>(0,0)=0.0; //k1;
                m_cam_distortion.at<double>(0,1)=0.0; //k2;
                m_cam_distortion.at<double>(0,2)=0.0; //t1;
                m_cam_distortion.at<double>(0,3)=0.0; //t2;
                m_cam_distortion.at<double>(0,4)=0.0; //k3;
            }
        }
        if (gCamera.check("remote"))
        {
            camera_remote = gCamera.find("remote").asString();
        }
    }

    //open the RGBDSensorClient
    yarp::os::Property rgbdOpts;
    rgbdOpts.put("device", "RGBDSensorClient");
    rgbdOpts.put("remoteImagePort", camera_remote + "/rgbImage:o");
    rgbdOpts.put("remoteDepthPort", camera_remote + "/depthImage:o");
    rgbdOpts.put("remoteRpcPort", camera_remote + "/rpc:i");
    rgbdOpts.put("localImagePort", "/" + getName() + "/rgbd/rgb");
    rgbdOpts.put("localDepthPort", "/" + getName() + "/rgbd/depth");
    rgbdOpts.put("localRpcPort", "/" + getName() + "/rgbd/rpc");
    rgbdOpts.put("ImageCarrier", "mjpeg");
    rgbdOpts.put("DepthCarrier", "fast_tcp");
    if (!m_rgbdDrv.open(rgbdOpts))
    {
        yError() << "Unable to open the RGBDSensorClient!";
        return false;
    }

    //open the gazebo interface
    m_simulation=rf.check("simulation",Value(false)).asBool();
    if (m_simulation)
    {
        m_gazeboPort.open("/" + getName() + "/gazebo:rpc");
    }

    //open the ports
    m_imgInPort.open("/" + getName() + "/img:i");
    m_imgOutPort.open("/" + getName() + "/img:o");
    m_opcPort.open("/" + getName() + "/opc:rpc");
    m_gazeStatePort.open("/" + getName() + "/gaze/state:i");
    m_navPort.open("/" + getName() + "/nav:rpc");
    m_viewerPort.open("/" + getName() + "/viewer:rpc");
    m_cmdPort.open("/" + getName() + "/cmd:rpc");
    attach(m_cmdPort);

    //open the tf driver
    yarp::os::Property tfOpts;
    tfOpts.put("device", "frameTransformClient");
    tfOpts.put("filexml_option", "ftc_pub_ros2.xml");
    if (m_tf.open(tfOpts))
    {
        if (m_tf.view(m_itf))
        {
            yInfo() << "FrameTransformClient successfully opened";
        }
        else
        {
            yWarning() << "Unable to view iFrameTransform interface";
        }
    }
    else
    {
        yWarning() << "Unable to open FrameTransformClient device driver";
    }

    return true;
}

/****************************************************************/
double Detector::getPeriod()
{
    return m_period;
}

/****************************************************************/
bool Detector::updateModule()
{
    std::lock_guard<std::mutex> lg(m_mtx_update);

    //configure camera
    if (!m_camera_configured)
    {
        m_camera_configured=getCameraOptions(m_cam_intrinsic, m_cam_distortion);
        return true;
    }

    update_cam_frame();
    update_nav_frame();

    if (m_simulation && !m_updated_line_viewer)
    {
        if (m_viewerPort.getOutputCount()>0)
        {
            if (update_line_sim("start-line") && update_line_sim("finish-line"))
            {
                if (opcAdd("start-line") && opcAdd("finish-line"))
                {
                    yarp::sig::Vector robot_pos;
                    get_model_pos("SIM_CER_ROBOT",robot_pos);
                    m_updated_line_viewer=create_line_viewer("start-line") &&
                                          create_line_viewer("finish-line") &&
                                          update_odometry(robot_pos[0],robot_pos[1],(180/M_PI)*robot_pos[6]);
                }
            }
        }
    }

    if (yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImg= m_imgInPort.read())
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImg= m_imgOutPort.prepare();
        cv::Mat inImgMat=yarp::cv::toCvMat(*inImg);

        if (opcCheck("start-line")<0 || opcCheck("finish-line")<0)
        {
            if (m_start_detection)
            {
                detect_helper(inImgMat);
            }
        }

        outImg=yarp::cv::fromCvMat<yarp::sig::PixelRgb>(inImgMat);
        m_imgOutPort.write();
    }
    return true;
}

/****************************************************************/
bool Detector::interruptModule()
{
    if (m_simulation)
    {
        m_gazeboPort.interrupt();
    }
    m_imgInPort.interrupt();
    m_imgOutPort.interrupt();
    m_opcPort.interrupt();
    m_gazeStatePort.interrupt();
    m_navPort.interrupt();
    m_viewerPort.interrupt();
    m_cmdPort.interrupt();
    return true;
}

/****************************************************************/
bool Detector::close()
{
    for(int i=0;i< m_nlines;i++)
    {
        delete m_lines_filter[i];
    }

    m_rgbdDrv.close();

    if (m_simulation)
    {
        m_gazeboPort.close();
    }

    m_imgInPort.close();
    m_imgOutPort.close();
    m_opcPort.close();
    m_gazeStatePort.close();
    m_navPort.close();
    m_viewerPort.close();
    m_cmdPort.close();
    return true;
}
