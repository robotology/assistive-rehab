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
    int nlines,line_filter_order;
    std::vector< cv::Ptr<cv::aruco::Dictionary> > dictionary;
    std::vector< cv::Ptr<cv::aruco::GridBoard> > board;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
    cv::Mat cam_intrinsic,cam_distortion;
    bool camera_configured;
    double fx,fy,px,py;
    cv::Vec3d rvec,tvec; // Rodrigues coefficients wrt cam
    std::map<std::string,int> line2idx;
    std::vector<int> nx,ny;
    std::vector<double> marker_size,marker_dist;

    std::vector<yarp::sig::Vector>lines_pose_world;
    std::vector<iCub::ctrl::MedianFilter*> lines_filter;
    std::mutex mtx_update,mtx_line_detected;
    std::condition_variable line_detected;
    yarp::sig::Matrix lineFrame,camFrame,navFrame,worldFrame;
    int line_cnt;
    std::string line;
    bool start_detection;
    bool updated_cam,updated_nav,updated_odom;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgOutPort;
    yarp::os::RpcClient camPort;
    yarp::os::RpcClient opcPort;
    yarp::os::RpcServer cmdPort;
    yarp::os::BufferedPort<yarp::os::Property> gazeStatePort;
    yarp::os::RpcClient navPort;
    yarp::os::RpcClient viewerPort;

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
        nlines=rf.check("nlines",Value(2)).asInt();
        line_filter_order=rf.check("line-filter-order",Value(30)).asInt();

        nx={6,6};
        if(Bottle *nxB=rf.find("nx").asList())
        {
            if(nxB->size()!=nlines)
            {
                yError()<<"The number of parameters has to match the number of lines";
                yError()<<"nx="<<nxB->toString();
                return false;
            }
            nx[0]=nxB->get(0).asInt();
            nx[1]=nxB->get(1).asInt();
        }

        ny={1,1};
        if(Bottle *nyB=rf.find("ny").asList())
        {
            if(nyB->size()!=nlines)
            {
                yError()<<"The number of parameters has to match the number of lines";
                yError()<<"ny="<<nyB->toString();
                return false;
            }
            ny[0]=nyB->get(0).asInt();
            ny[1]=nyB->get(1).asInt();
        }

        marker_size={0.13,0.13};
        if(Bottle *mSzB=rf.find("marker-size").asList())
        {
            if(mSzB->size()!=nlines)
            {
                yError()<<"The number of parameters has to match the number of lines";
                yError()<<"marker-size="<<mSzB->toString();
                return false;
            }
            marker_size[0]=mSzB->get(0).asDouble();
            marker_size[1]=mSzB->get(1).asDouble();
        }

        marker_dist={0.005,0.005};
        if(Bottle *mDistB=rf.find("marker-dist").asList())
        {
            if(mDistB->size()!=nlines)
            {
                yError()<<"The number of parameters has to match the number of lines";
                yError()<<"marker-dist="<<mDistB->toString();
                return false;
            }
            marker_dist[0]=mDistB->get(0).asDouble();
            marker_dist[1]=mDistB->get(1).asDouble();
        }

        line2idx["start-line"]=0;
        line2idx["finish-line"]=1;

        //create dictionary
        //0 corresponds to the starting line
        //1 corresponds to the ending line
        dictionary.resize(nlines);
        dictionary[0]=cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
        dictionary[1]=cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);

        board.resize(nlines);
        lines_pose_world.resize(nlines);
        lines_filter.resize(nlines);
        for(int i=0; i<nlines;i++)
        {
            board[i]=cv::aruco::GridBoard::create(nx[i],ny[i],marker_size[i],marker_dist[i],dictionary[i]);
            lines_pose_world[i]=yarp::sig::Vector(7,0.0);
            lines_filter[i]=new iCub::ctrl::MedianFilter(line_filter_order,yarp::sig::Vector(7,0.0));
        }

        //configure the detector with corner refinement
        detector_params = cv::aruco::DetectorParameters::create();
        detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        start_detection=false;
        updated_cam=updated_nav=updated_odom=false;
        camera_configured=false;
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

        imgInPort.open("/" + getName() + "/img:i");
        imgOutPort.open("/" + getName() + "/img:o");
        camPort.open("/" + getName() + "/cam:rpc");
        opcPort.open("/" + getName() + "/opc:rpc");
        gazeStatePort.open("/" + getName() + "/gaze/state:i");
        navPort.open("/" + getName() + "/nav:rpc");
        viewerPort.open("/" + getName() + "/viewer:rpc");
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

        //configure camera
        if (!camera_configured)
        {
            camera_configured=getCameraOptions(cam_intrinsic,cam_distortion);
            return true;
        }

        update_cam_frame();
        update_nav_frame();

        if (yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImg=imgInPort.read())
        {
            yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImg=imgOutPort.prepare();
            cv::Mat inImgMat=yarp::cv::toCvMat(*inImg);

            if (opcCheck("start-line")<0 || opcCheck("finish-line")<0)
            {
                if (start_detection)
                {
                    detect_helper(inImgMat);
                }
            }

            outImg=yarp::cv::fromCvMat<yarp::sig::PixelRgb>(inImgMat);
            imgOutPort.write();
        }
        return true;
    }

    /****************************************************************/
    int opcCheck(const std::string &line_tag)
    {
        int id=-1;
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("ask"));

            Bottle &bLine=cmd.addList();
            Bottle &b1=bLine.addList();
            b1.addString(line_tag);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    if (Bottle *idB=rep.get(1).asList())
                    {
                        if (Bottle *idList=idB->get(1).asList())
                        {
                            if (idList->size()>0)
                            {
                                id=idList->get(0).asInt();
                                get_line_opc(id,line_tag);
                            }
                        }
                    }
                }
            }
        }
        return id;
    }

    /****************************************************************/
    yarp::os::Property get_line_opc(const int id, const std::string &line_tag)
    {
        Property prop;
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("get"));

            Bottle &l=cmd.addList();
            l.addString("id");
            l.addInt(id);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ack"))
                {
                    Property lineProp(rep.get(1).toString().c_str());
                    if (Bottle *b=lineProp.find(line_tag).asList())
                    {
                        prop.fromString(b->toString().c_str());
                    }
                }
            }
        }
        return prop;
    }

    /****************************************************************/
    bool opcAdd(const yarp::sig::Vector &pose_world, const yarp::sig::Vector &line_size,
                const int opc_id, const std::string &line_tag)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("add"));

            Bottle bPoseWorld;
            Property poseProp;
            bPoseWorld.addList().read(pose_world);
            poseProp.put("pose_world",bPoseWorld.get(0));

            Bottle bSize;
            bSize.addList().read(line_size);
            poseProp.put("size",bSize.get(0));

            Property prop;
            Bottle line;
            line.addList().read(poseProp);
            prop.put(line_tag,line.get(0));

            if(opc_id<0)
            {
                cmd.addList().read(prop);
                if (opcPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==Vocab::encode("ack"))
                    {
                        int id=rep.get(1).asList()->get(1).asInt();
                        return opcSet(prop,id);
                    }
                }
            }
            else
            {
                return opcSet(prop,opc_id);
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
    bool detect(const std::string &line, const int timeout) override
    {
        std::unique_lock<std::mutex> lck(mtx_line_detected);
        if (opcCheck(line)>=0)
        {
            yWarning()<<"Line already inside opc";
            return false;
        }
        this->line=line;
        if(line2idx.count(line)<=0)
        {
            yWarning()<<"This line does not exist";
            return false;
        }
        start_detection=true;
        line_cnt=0;
        yInfo()<<"Detecting"<<line;
        if (timeout>0)
        {
            line_detected.wait_for(lck,std::chrono::seconds(timeout));
        }
        else
        {
            line_detected.wait(lck);
        }

        if (line=="start-line" && !updated_odom)
        {
            yarp::sig::Matrix wf=SE3inv(camFrame*lineFrame);
            yarp::sig::Vector tr=wf.getCol(3);
            yarp::sig::Vector rot=(180/M_PI)*dcm2axis(wf.submatrix(0,2,0,2));
            update_odometry(tr[0],tr[1],rot[3]);
        }
        create_line(line);
        start_detection=false;

        return true;
    }

    /****************************************************************/
    bool reset() override
    {
      std::lock_guard<std::mutex> lg(mtx_update);
      for(int i=0;i<nlines;i++)
      {
          lines_pose_world[i]=yarp::sig::Vector(7,0.0);
          lines_filter[i]->init(yarp::sig::Vector(7,0.0));
          updated_odom=false;
          std::string l="";
          for (auto &j:line2idx)
          {
              if(j.second==i)
              {
                  l=j.first;
              }
          }
          remove_from_viewer(l);
          int opc_id=opcCheck(l);
          if (opc_id>=0)
          {
              opcDel(opc_id);
          }

      }
      line_detected.notify_all();
      return true;
    }

    /****************************************************************/
    void create_line(const std::string &line_tag)
    {
        int i=line2idx[line_tag];
        double llen=(marker_dist[i]+marker_size[i])*nx[i];
        yarp::sig::Vector ax(4);
        ax[0]=lines_pose_world[i][3];
        ax[1]=lines_pose_world[i][4];
        ax[2]=lines_pose_world[i][5];
        ax[3]=lines_pose_world[i][6];
        yarp::sig::Matrix R=axis2dcm(ax);
        yarp::sig::Vector u=R.subcol(0,0,2);
        yarp::sig::Vector p0(3);
        p0[0]=lines_pose_world[i][0];
        p0[1]=lines_pose_world[i][1];
        p0[2]=lines_pose_world[i][2];
        yarp::sig::Vector p1=p0+llen*u;
        std::vector<int> color(3,0.0);
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
        cmd.addDouble(p0[0]);
        cmd.addDouble(p0[1]);
        cmd.addDouble(p0[2]);
        cmd.addDouble(p1[0]);
        cmd.addDouble(p1[1]);
        cmd.addDouble(p1[2]);
        cmd.addInt(color[0]);
        cmd.addInt(color[1]);
        cmd.addInt(color[2]);
        if(viewerPort.write(cmd,rep))
        {
            if(rep.get(0).asBool()==true)
            {
                yInfo()<<line_tag<<"created on the viewer";
            }
        }
    }

    /****************************************************************/
    void remove_from_viewer(const std::string &l)
    {
        Bottle cmd,rep;
        cmd.addString("delete_line");
        cmd.addString(l);
        if(viewerPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab()==Vocab::encode("ok"))
            {
                yInfo()<<l<<"deleted";
            }
        }
    }

    /****************************************************************/
    bool detect_helper(const cv::Mat &inImgMat)
    {
        //detect markers
        int line_idx=line2idx.at(line);
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

                if (line=="start-line")
                {
                    lineFrame.resize(4,4);
                    lineFrame.setSubmatrix(R,0,0);
                    lineFrame.setCol(3,pose);
                    lineFrame.setRow(3,yarp::sig::Vector(3,0.0));
                    yarp::sig::Matrix T=SE3inv(camFrame*lineFrame);
                    worldFrame=T*camFrame;
                }

                if (line=="finish-line")
                {
                    worldFrame=navFrame*camFrame;
                }

                yarp::sig::Vector rot=yarp::math::dcm2axis(worldFrame.submatrix(0,2,0,2));
                yarp::sig::Vector est_pose_world(7,0.0);
                est_pose_world.setSubvector(0,worldFrame*pose);
                est_pose_world.setSubvector(3,rot);

                lines_pose_world[line_idx]=lines_filter[line_idx]->filt(est_pose_world);
                yarp::sig::Vector v=lines_pose_world[line_idx].subVector(3,5);
                if (norm(v)>0.0)
                    v/=norm(v);
                lines_pose_world[line_idx].setSubvector(3,v);
                line_cnt++;

                if (line_cnt>line_filter_order)
                {
                    int opc_id=opcCheck(line);
                    yarp::sig::Vector line_size(2);
                    line_size[0]=(marker_dist[line_idx]+marker_size[line_idx])*nx[line_idx];
                    line_size[1]=marker_size[line_idx]*ny[line_idx];
                    opcAdd(lines_pose_world[line_idx],line_size,opc_id,line);
                    line_detected.notify_all();
                    return true;
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
    bool update_cam_frame()
    {
        if (Property *p=gazeStatePort.read(false))
        {
            yarp::sig::Vector pose;
            p->find("depth_rgb").asList()->write(pose);
            camFrame=yarp::math::axis2dcm(pose.subVector(3,6));
            camFrame.setSubcol(pose.subVector(0,2),0,3);
            updated_cam=true;
            return true;
        }
        return false;
    }

    /****************************************************************/
    bool update_nav_frame()
    {
        Bottle cmd,rep;
        cmd.addString("get_state");
        if (navPort.write(cmd,rep))
        {
            Property robotState(rep.get(0).toString().c_str());
            if (Bottle *loc=robotState.find("robot-location").asList())
            {
                yarp::sig::Vector robot_location(7,0.0);
                robot_location[0]=loc->get(0).asDouble();
                robot_location[1]=loc->get(1).asDouble();
                robot_location[5]=1.0;
                robot_location[6]=(M_PI/180)*loc->get(2).asDouble();
                navFrame=yarp::math::axis2dcm(robot_location.subVector(3,6));
                navFrame.setSubcol(robot_location.subVector(0,2),0,3);
                updated_nav=true;
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool update_odometry(const double x, const double y, const double theta)
    {
        Bottle cmd,rep;
        cmd.addString("reset_odometry");
        cmd.addDouble(x);
        cmd.addDouble(y);
        cmd.addDouble(theta);
        yDebug()<<cmd.toString();
        if (navPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab()==Vocab::encode("ok"))
            {
                updated_odom=true;
                yInfo()<<"Reset robot's odometry";
                return true;
            }
        }
        return false;
    }

    /****************************************************************/
    bool update_odometry(const std::string &line_tag, const double theta) override
    {
        std::lock_guard<std::mutex> lg(mtx_update);
        if (line2idx.count(line_tag)<=0)
        {
            return false;
        }
        int i=line2idx[line_tag];
        yarp::sig::Vector pose=lines_pose_world[i].subVector(0,2);
        return update_odometry(pose[0],pose[1],theta);
    }

    /****************************************************************/
    bool go_to_line(const std::string &line_tag, const double theta) override
    {
        std::lock_guard<std::mutex> lg(mtx_update);
        if (updated_odom && line2idx.count(line_tag)>0)
        {
            yInfo()<<"Going to"<<line_tag;
            int i=line2idx[line_tag];
            yarp::sig::Vector pose=lines_pose_world[i].subVector(0,2);
            Bottle cmd,rep;
            cmd.addString("go_to_wait");
            cmd.addDouble(pose[0]);
            cmd.addDouble(pose[1]);
            cmd.addDouble(theta);
            if (navPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab()==Vocab::encode("ok"))
                {
                    yInfo()<<"Reached"<<line_tag;
                    cmd.clear();
                    rep.clear();
                    cmd.addString("reset_odometry");
                    cmd.addDouble(pose[0]);
                    cmd.addDouble(pose[1]);
                    cmd.addDouble(theta);
                    yDebug()<<cmd.toString();
                    if (navPort.write(cmd,rep))
                    {
                        if (rep.get(0).asVocab()==Vocab::encode("ok"))
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

    /****************************************************************/
    yarp::os::Property get_line(const std::string &line) override
    {
        std::lock_guard<std::mutex> lg(mtx_update);
        if(line2idx.count(line)>0)
        {
            int id=opcCheck(line);
            if (id>=0)
            {
                return get_line_opc(id,line);
            }
            else
            {
                return {};
            }
        }
        else
        {
            yWarning()<<"This line does not exist";
            return {};
        }
    }

    /****************************************************************/
    bool interruptModule() override
    {
        imgInPort.interrupt();
        imgOutPort.interrupt();
        camPort.interrupt();
        opcPort.interrupt();
        gazeStatePort.interrupt();
        navPort.interrupt();
        viewerPort.interrupt();
        cmdPort.interrupt();
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        for(int i=0;i<nlines;i++)
        {
            delete lines_filter[i];
        }

        imgInPort.close();
        imgOutPort.close();
        camPort.close();
        opcPort.close();
        gazeStatePort.close();
        navPort.close();
        viewerPort.close();
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
                    fx = intrinsics->find("focalLengthX").asDouble();
                    fy = intrinsics->find("focalLengthY").asDouble();
                    px = intrinsics->find("principalPointX").asDouble();
                    py = intrinsics->find("principalPointY").asDouble();

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
