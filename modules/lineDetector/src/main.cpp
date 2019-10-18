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
#include <yarp/dev/IVisualParams.h>
#include <yarp/dev/GenericVocabs.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/filters.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include "src/lineDetector_IDL.h"

using namespace yarp::os;
using namespace yarp::math;

class Detector : public RFModule, public lineDetector_IDL
{
    double period;
    int line_filter_order;
    std::vector< cv::Ptr<cv::aruco::Dictionary> > dictionary;
    std::vector< cv::Ptr<cv::aruco::GridBoard> > board;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
    cv::Mat cam_intrinsic,cam_distortion;
    std::vector<int> opc_id;
    std::vector<bool> add;
    bool camera_configured;
    double fx,fy,px,py;
    cv::Vec3d rvec,tvec; // Rodrigues coefficients wrt cam

    std::vector<yarp::sig::Vector>lines_pose;
    std::vector<iCub::ctrl::MedianFilter*> lines_filter;
    std::mutex mtx_update,mtx_line_detected;
    std::condition_variable line_detected;
    yarp::sig::Matrix cam2root,cam2world,root2world,robotT;
    int line_idx,line_cnt;
    bool line_estimated;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgOutPort;
    yarp::os::RpcClient camPort;
    yarp::os::RpcClient opcPort;
    yarp::os::RpcServer cmdPort;
    yarp::os::BufferedPort<yarp::os::Property> gazeStatePort;
    yarp::os::RpcClient navPort;

    /****************************************************************/
    bool attach(RpcServer &source) override
    {
        return yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        //parameters
        std::string moduleName=rf.check("name", Value("lineDetector")).asString();
        setName(moduleName.c_str());

        period=rf.check("period",Value(0.1)).asDouble();
        line_filter_order=rf.check("line-filter-order",Value(30)).asInt();

        std::vector<int> nx={6,6};
        if(Bottle *nxB=rf.find("nx").asList())
        {
            nx[0]=nxB->get(0).asInt();
            nx[1]=nxB->get(1).asInt();
        }

        std::vector<int> ny={1,1};
        if(Bottle *nyB=rf.find("ny").asList())
        {
            ny[0]=nyB->get(0).asInt();
            ny[1]=nyB->get(1).asInt();
        }

        std::vector<double> marker_size={0.13,0.13};
        if(Bottle *mSzB=rf.find("marker-size").asList())
        {
            marker_size[0]=mSzB->get(0).asDouble();
            marker_size[1]=mSzB->get(1).asDouble();
        }

        std::vector<double> marker_dist={0.005,0.005};
        if(Bottle *mDistB=rf.find("marker-dist").asList())
        {
            marker_dist[0]=mDistB->get(0).asDouble();
            marker_dist[1]=mDistB->get(1).asDouble();
        }

        //create dictionary
        //0 corresponds to the starting line
        //1 corresponds to the ending line
        dictionary.resize(2);
        dictionary[0]=cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
        dictionary[1]=cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);

        board.resize(2);
        board[0]=cv::aruco::GridBoard::create(nx[0],ny[0],marker_size[0],marker_dist[0],dictionary[0]);
        board[1]=cv::aruco::GridBoard::create(nx[1],ny[1],marker_size[1],marker_dist[1],dictionary[1]);

        //configure the detector with corner refinement
        detector_params = cv::aruco::DetectorParameters::create();
        detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        add.resize(2);
        add[0]=true;
        add[1]=true;
        opc_id.resize(2);
        opc_id[0]=-1;
        opc_id[1]=-1;
        camera_configured=false;
        line_idx=-1;
        line_estimated=false;

        cam_intrinsic=cv::Mat::eye(3,3,CV_64F);
        cam_distortion=cv::Mat::zeros(1,5,CV_64F);
        Bottle &gCamera=rf.findGroup("camera");
        if (!gCamera.isNull())
        {
            if (Bottle *intrinsics=gCamera.find("intrinsics").asList())
            {
                if (intrinsics->size()>=4)
                {
                    camera_configured=true;
                    fx=intrinsics->get(0).asDouble();
                    fy=intrinsics->get(1).asDouble();
                    px=intrinsics->get(2).asDouble();
                    py=intrinsics->get(3).asDouble();
                    yInfo()<<"camera fx (from file) ="<<fx;
                    yInfo()<<"camera fy (from file) ="<<fy;
                    yInfo()<<"camera px (from file) ="<<px;
                    yInfo()<<"camera py (from file) ="<<py;

                    cam_intrinsic.at<double>(0,0)=fx;
                    cam_intrinsic.at<double>(0,2)=px;
                    cam_intrinsic.at<double>(1,1)=fy;
                    cam_intrinsic.at<double>(1,2)=py;

                    cam_distortion.at<double>(0,0)=0.0; //k1;
                    cam_distortion.at<double>(0,1)=0.0; //k2;
                    cam_distortion.at<double>(0,2)=0.0; //t1;
                    cam_distortion.at<double>(0,3)=0.0; //t2;
                    cam_distortion.at<double>(0,4)=0.0; //k3;
                }
            }
        }

        for(int i=0; i<2;i++)
        {
            lines_pose.push_back(yarp::sig::Vector(7,0.0));
            lines_filter.push_back(new iCub::ctrl::MedianFilter(line_filter_order,yarp::sig::Vector(7,0.0)));
        }

        imgInPort.open("/" + getName() + "/img:i");
        imgOutPort.open("/" + getName() + "/img:o");
        camPort.open("/" + getName() + "/cam:rpc");
        opcPort.open("/" + getName() + "/opc:rpc");
        gazeStatePort.open("/" + getName() + "/gaze/state:i");
        navPort.open("/" + getName() + "/nav:rpc");
        cmdPort.open("/" + getName() + "/cmd:rpc");
        attach(cmdPort);

        return true;
    }

    /****************************************************************/
    double getPeriod() override
    {
        return period;
    }

    /****************************************************************/
    bool updateModule() override
    {
        std::lock_guard<std::mutex> lg(mtx_update);
        if (Property *p=gazeStatePort.read(false))
        {
            yarp::sig::Vector pose;
            p->find("depth_rgb").asList()->write(pose);
            cam2root=yarp::math::axis2dcm(pose.subVector(3,6));
            cam2root.setSubcol(pose.subVector(0,2),0,3);

            Bottle cmd,rep;
            cmd.addString("get_state");
            if (navPort.write(cmd,rep))
            {
                Property robotState(rep.get(0).toString().c_str());
                if (Bottle *loc=robotState.find("robot-location").asList())
                {
                    yarp::sig::Vector robot_location(4);
                    robot_location[0]=loc->get(0).asDouble();
                    robot_location[1]=loc->get(1).asDouble();
                    robot_location[2]=0.0;
                    robot_location[3]=loc->get(2).asDouble();

                    yarp::sig::Matrix T(4,4);
                    yarp::sig::Vector rot(4,0.0);
                    rot[2]=1.0;
                    rot[3]=robot_location[3];
                    T=yarp::math::axis2dcm(rot);
                    T.setSubcol(robot_location.subVector(0,2),0,3);
                    robotT=SE3inv(T);
                }
            }

            //configure camera
            if (!camera_configured)
            {
                camera_configured=getCameraOptions(cam_intrinsic,cam_distortion);
                return true;
            }

            if (yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImg=imgInPort.read())
            {
                yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImg=imgOutPort.prepare();
                cv::Mat inImgMat=yarp::cv::toCvMat(*inImg);

                if (line_idx>=0)
                {
                    line_estimated=detect_helper(inImgMat);
                }

                outImg=yarp::cv::fromCvMat<yarp::sig::PixelRgb>(inImgMat);
                imgOutPort.write();
            }
        }
        return true;
    }

    /****************************************************************/
    bool opcAdd(const yarp::sig::Vector &pose_world, const bool &add, const int idx)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("add"));

            Bottle bPoseWorld;
            Property poseProp;
            bPoseWorld.addList().read(pose_world);
            poseProp.put("pose_world",bPoseWorld.get(0));

            Property prop;
            Bottle line;
            line.addList().read(poseProp);
            if(idx==0)
            {
                prop.put("start-line",line.get(0));
            }
            else if(idx==1)
            {
                prop.put("finish-line",line.get(0));
            }

            if(add)
            {
                cmd.addList().read(prop);
                if (opcPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==Vocab::encode("ack"))
                    {
                        opc_id[idx]=rep.get(1).asList()->get(1).asInt();
                        return opcSet(prop,opc_id[idx]);
                    }
                }
            }
            else
            {
                return opcSet(prop,opc_id[idx]);
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcSet(const yarp::os::Property &prop, const int id_line)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("set"));
            Bottle &pl=cmd.addList();
            pl.read(prop);
            Bottle id;
            Bottle &id_pl=id.addList();
            id_pl.addString("id");
            id_pl.addInt(id_line);
            pl.append(id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab()==Vocab::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcDel(const int id)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("del"));
            Bottle &pl=cmd.addList().addList();
            pl.addString("id");
            pl.addInt(id);
            if (opcPort.write(cmd,rep))
            {
                if(rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    yInfo()<<"Line with id"<<id<<"removed from opc";
                    return true;
                }
            }
        }

        return false;
    }

    /****************************************************************/
    bool detect(const int line_idx, const int timeout) override
    {
        std::unique_lock<std::mutex> lck(mtx_line_detected);
        this->line_idx=line_idx;
        line_cnt=0;
        yInfo()<<"Detecting line"<<line_idx;
        const int ok=Vocab::encode("ok");
        if (line_idx==0)
        {
            Bottle cmd,rep;
            cmd.addString("reset_odometry");
            if (navPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==ok)
                {
                    yInfo()<<"Robot at 0.0 0.0 0.0";
                }
            }
        }
        if (timeout>0)
        {
            line_detected.wait_for(lck,std::chrono::seconds(timeout));
        }
        else
        {
            line_detected.wait(lck);
        }
        return line_estimated;
    }

    /****************************************************************/
    yarp::sig::Matrix get_world_frame() override
    {
        std::lock_guard<std::mutex> lg(mtx_update);
        return root2world;
    }

    /****************************************************************/
    bool reset() override
    {
      std::lock_guard<std::mutex> lg(mtx_update);
      line_idx=-1;
      line_estimated=false;
      cam2world.zero();
      root2world.zero();
      for(int i=0;i<2;i++)
      {
          lines_pose[i]=yarp::sig::Vector(7,0.0);
          lines_filter[i]->init(yarp::sig::Vector(7,0.0));
          opcDel(opc_id[i]);
          add[i]=true;
          opc_id[i]=true;
      }
      line_detected.notify_all();
      return true;
    }

    /****************************************************************/
    bool stop() override
    {
      std::lock_guard<std::mutex> lg(mtx_update);
      line_idx=-1;
      line_detected.notify_all();
      yInfo()<<"Stopping detection";
      return true;
    }

    /****************************************************************/
    bool detect_helper(const cv::Mat &inImgMat)
    {
        //detect markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        cv::aruco::detectMarkers(inImgMat,dictionary[line_idx],corners,ids);

        if (ids.size()>0)
        {
            std::vector<std::vector<cv::Point2f> > rejected;
            cv::Ptr<cv::aruco::GridBoard> curr_board=board[line_idx];

            //perform refinement
            cv::aruco::refineDetectedMarkers(inImgMat,curr_board,corners,ids,rejected,
                                             cam_intrinsic,cam_distortion);

            cv::aruco::drawDetectedMarkers(inImgMat,corners,ids);

            //estimate pose
            //use cv::Mat and not cv::Vec3d to avoid issues with
            //cv::aruco::estimatePoseBoard() initial guess
            int valid=cv::aruco::estimatePoseBoard(corners,ids,curr_board,
                                                   cam_intrinsic,cam_distortion,
                                                   rvec,tvec,true);

            //if at least one board marker detected
            if (valid>0)
            {
                cv::aruco::drawAxis(inImgMat,cam_intrinsic,cam_distortion,rvec,tvec,0.5);
                cv::Mat Rmat;
                cv::Rodrigues(rvec,Rmat);
                yarp::sig::Matrix R(3,3);
                for(int i=0;i<Rmat.rows;i++)
                {
                    for(int j=0;j<Rmat.cols;j++)
                    {
                        R(i,j)=Rmat.at<double>(i,j);
                    }
                }

                yarp::sig::Vector pose(4);
                pose[0]=tvec[0];
                pose[1]=tvec[1];
                pose[2]=tvec[2];
                pose[3]=1.0;

                if (line_idx==0)
                {
                    yarp::sig::Matrix world2cam(4,4);
                    world2cam.setSubmatrix(R,0,0);
                    world2cam.setCol(3,pose);
                    world2cam.setRow(3,yarp::sig::Vector(3,0.0));
                    cam2world=SE3inv(world2cam);
                    root2world=SE3inv(cam2root*world2cam);
                }

                if (cam2world.data())
                {
                    yarp::sig::Vector rot=yarp::math::dcm2axis(root2world*robotT*cam2root.submatrix(0,2,0,2));
                    yarp::sig::Vector est_pose_world(7,0.0);
                    est_pose_world.setSubvector(0,root2world*robotT*cam2root*pose);
                    est_pose_world.setSubvector(3,rot);

                    lines_pose[line_idx]=lines_filter[line_idx]->filt(est_pose_world);
                    yarp::sig::Vector v=lines_pose[line_idx].subVector(3,5);
                    if (norm(v)>0.0)
                        v/=norm(v);
                    lines_pose[line_idx].setSubvector(3,v);
                    line_cnt++;

                    if (line_cnt>line_filter_order)
                    {
                        yInfo()<<"Line"<<line_idx<<"detected at"<<lines_pose[line_idx].toString();
                        bool add_to_opc=add[line_idx];
                        opcAdd(lines_pose[line_idx],add_to_opc,line_idx);
                        if(add_to_opc && opc_id[line_idx]!=-1)
                        {
                            add[line_idx]=false;
                        }
                        line_detected.notify_all();
                        return true;
                    }
                }
                else
                {
                    yWarning()<<"We need to estimate start-line first";
                    line_detected.notify_all();
                }
            }
            else
            {
                yInfo()<<"No board marker detected";
            }
        }
        else
        {
            yInfo()<<"Line not in FOV";
        }
        return false;
    }

    /****************************************************************/
    yarp::sig::Vector get_line_pose(const int line_idx) override
    {
        std::lock_guard<std::mutex> lg(mtx_update);
        if (line_estimated)
            return lines_pose[line_idx];
        else
            return {};
    }

    /****************************************************************/
    bool interruptModule() override
    {
        for(int i=0;i<opc_id.size();i++)
            opcDel(opc_id[i]);

        imgInPort.interrupt();
        imgOutPort.interrupt();
        camPort.interrupt();
        opcPort.interrupt();
        gazeStatePort.interrupt();
        navPort.interrupt();
        cmdPort.interrupt();
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        for(int i=0;i<2;i++)
        {
            delete lines_filter[i];
        }

        imgInPort.close();
        imgOutPort.close();
        camPort.close();
        opcPort.close();
        gazeStatePort.close();
        navPort.close();
        cmdPort.close();
        return true;
    }

    /****************************************************************/
    bool getCameraOptions(cv::Mat &cam_intrinsic, cv::Mat &cam_distortion)
    {
        if (camPort.getOutputCount()>0)
        {
            yInfo()<<"Reading intrinsics from camera";
            Bottle cmd,rep;
            cmd.addVocab(VOCAB_RGB_VISUAL_PARAMS);
            cmd.addVocab(VOCAB_GET);
            cmd.addVocab(VOCAB_INTRINSIC_PARAM);
            if (camPort.write(cmd,rep))
            {
                if (rep.size()>=4)
                {
                    yarp::os::Bottle *intrinsics = rep.get(3).asList();
                    yarp::os::Bottle *focal_x = intrinsics->get(1).asList();
                    yarp::os::Bottle *focal_y = intrinsics->get(2).asList();
                    yarp::os::Bottle *radial_1 = intrinsics->get(3).asList();
                    yarp::os::Bottle *radial_2 = intrinsics->get(4).asList();
                    yarp::os::Bottle *radial_3 = intrinsics->get(5).asList();
                    yarp::os::Bottle *principal_x = intrinsics->get(7).asList();
                    yarp::os::Bottle *principal_y = intrinsics->get(8).asList();
                    yarp::os::Bottle *tangential_1 = intrinsics->get(10).asList();
                    yarp::os::Bottle *tangential_2 = intrinsics->get(11).asList();

                    fx = focal_x->get(1).asDouble();
                    fy = focal_y->get(1).asDouble();
                    double k1 = radial_1->get(1).asDouble();
                    double k2 = radial_2->get(1).asDouble();
                    double k3 = radial_3->get(1).asDouble();
                    double t1 = tangential_1->get(1).asDouble();
                    double t2 = tangential_2->get(1).asDouble();
                    px = principal_x->get(1).asDouble();
                    py = principal_y->get(1).asDouble();

                    yInfo()<<"Camera intrinsics:"<<fx<<fy<<px<<py;

                    cam_intrinsic.at<double>(0,0)=fx;
                    cam_intrinsic.at<double>(0,2)=px;
                    cam_intrinsic.at<double>(1,1)=fy;
                    cam_intrinsic.at<double>(1,2)=py;
                    cam_intrinsic.at<double>(2,2)=1.0;

                    cam_distortion.at<double>(0,0)=0.0; //k1;
                    cam_distortion.at<double>(0,1)=0.0; //k2;
                    cam_distortion.at<double>(0,2)=0.0; //t1;
                    cam_distortion.at<double>(0,3)=0.0; //t2;
                    cam_distortion.at<double>(0,4)=0.0; //k3;

                    return true;
                }
            }
        }
        return false;
    }

};



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
