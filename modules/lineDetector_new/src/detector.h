#ifndef DETECTOR_H
#define DETECTOR_H

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
#include <yarp/dev/IFrameTransform.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/filters.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include "src/lineDetector_IDL.h"

using namespace yarp::os;
using namespace yarp::math;

class Detector : public RFModule, public lineDetector_IDL
{
    double m_period;
    int m_nlines;
    int m_line_filter_order;
    std::vector< cv::Ptr <cv::aruco::Dictionary> > m_dictionary;
    std::vector< cv::Ptr <cv::aruco::GridBoard> > m_board;
    cv::aruco::DetectorParameters m_detector_params;
    cv::Mat m_cam_intrinsic, m_cam_distortion;
    bool m_camera_configured;
    double m_fx, m_fy, m_px, m_py;
    cv::Vec3d m_rvec, m_tvec; // Rodrigues coefficients wrt cam
    std::vector<cv::Vec3d> m_rvec_v;
    std::vector<cv::Vec3d> m_tvec_v;
    std::map<std::string, int> m_line2idx;
    std::vector<int> m_nx, m_ny;
    std::vector<double> m_marker_size, m_marker_dist;
    bool m_simulation, m_updated_line_viewer, m_use_initial_guess;

    std::vector<yarp::sig::Vector> m_lines_pose_world;
    std::vector<yarp::sig::Vector> m_lines_size;
    std::vector<iCub::ctrl::MedianFilter*> m_lines_filter;
    std::mutex m_mtx_update;
    std::mutex m_mtx_line_detected;
    std::condition_variable m_line_detected;
    yarp::sig::Matrix m_lineFrame, m_camFrame, m_navFrame, m_worldFrame;
    int m_line_cnt;
    std::string m_line;
    bool m_start_detection;
    bool m_updated_cam, m_updated_nav, m_updated_odom;

    yarp::dev::PolyDriver m_rgbdDrv;
    yarp::dev::PolyDriver m_tf;
    yarp::dev::IFrameTransform* m_itf=nullptr;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_imgInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_imgOutPort;
    yarp::os::RpcClient m_opcPort;
    yarp::os::RpcServer m_cmdPort;
    yarp::os::BufferedPort<yarp::os::Property> m_gazeStatePort;
    yarp::os::RpcClient m_navPort;
    yarp::os::RpcClient m_viewerPort;
    yarp::os::RpcClient m_gazeboPort;

    /****************************************************************/
    //RFModule
    bool attach(RpcServer& source) override;
    bool configure(ResourceFinder& rf) override;
    double getPeriod() override;
    bool updateModule() override;
    bool interruptModule() override;
    bool close() override;

    /****************************************************************/
    //lineDetector_IDL
    bool detect(const std::string& line, const int timeout) override;
    bool reset() override;
    bool update_odometry(const std::string& line_tag, const double theta) override;
    bool go_to_line(const std::string& line_tag, const double theta) override;
    yarp::os::Property get_line(const std::string& line) override;
    bool set_initial_guess(bool flag) override;
    bool get_initial_guess() override;

    /****************************************************************/
    //simulation only
    bool get_model_pos(const std::string& model_name, yarp::sig::Vector& v);
    bool update_line_sim(const std::string& line_str);

    /****************************************************************/
    //opc stuff
    int opcCheck(const std::string& line_tag);
    yarp::os::Property opcGetLine(const int id, const std::string& line_tag);
    bool opcAdd(const std::string& line_tag);
    bool opcAdd(const yarp::sig::Vector& pose_world, const yarp::sig::Vector& line_size, const int opc_id, const std::string& line_tag);
    bool opcSet(const yarp::os::Property& prop, const int id_line);
    bool opcDel(const int id);

    /****************************************************************/
    //viewer
    bool create_line_viewer(const std::string& line_tag);
    void delete_line_viewer(const std::string& l);
    void create_line_tf(const std::string& line_tag);
    void delete_line_tf(const std::string& l);

    /****************************************************************/
    yarp::sig::Matrix toYarpMat(const cv::Mat& Rmat);
    yarp::sig::Matrix update_world_frame(const std::string& line, const yarp::sig::Matrix& R, const yarp::sig::Vector& pose);
    void estimate_line(const yarp::sig::Matrix& wf, const yarp::sig::Vector& pose, const int line_idx);
    bool detect_helper(const cv::Mat& inImgMat);
    bool update_cam_frame();
    bool update_nav_frame();
    bool update_odometry(const double x, const double y, const double theta);

    bool getCameraOptions(cv::Mat& cam_intrinsics, cv::Mat& cam_distortion);



public:

    /****************************************************************/
    Detector() : m_start_detection(false), m_updated_cam(false), m_updated_nav(false),
        m_updated_odom(false), m_camera_configured(false), m_updated_line_viewer(false),
        m_lineFrame(eye(4)), m_camFrame(eye(4)), m_navFrame(eye(4)), m_worldFrame(eye(4)) { }

};

#endif
