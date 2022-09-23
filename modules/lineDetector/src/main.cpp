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
    std::vector<cv::Vec3d> rvec_v;
    std::vector<cv::Vec3d> tvec_v;
    std::map<std::string,int> line2idx;
    std::vector<int> nx,ny;
    std::vector<double> marker_size,marker_dist;
    bool simulation,updated_line_viewer, use_initial_guess;

    std::vector<yarp::sig::Vector>lines_pose_world;
    std::vector<yarp::sig::Vector>lines_size;
    std::vector<iCub::ctrl::MedianFilter*> lines_filter;
    std::mutex mtx_update,mtx_line_detected;
    std::condition_variable line_detected;
    yarp::sig::Matrix lineFrame,camFrame,navFrame,worldFrame;
    int line_cnt;
    std::string line;
    bool start_detection;
    bool updated_cam,updated_nav,updated_odom;

    yarp::dev::PolyDriver rgbdDrv;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgOutPort;
    yarp::os::RpcClient opcPort;
    yarp::os::RpcServer cmdPort;
    yarp::os::BufferedPort<yarp::os::Property> gazeStatePort;
    yarp::os::RpcClient navPort;
    yarp::os::RpcClient viewerPort;
    yarp::os::RpcClient gazeboPort;

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

        period=rf.check("period",Value(0.1)).asFloat64();
        nlines=rf.check("nlines",Value(2)).asInt32();
        line_filter_order=rf.check("line-filter-order",Value(30)).asInt32();
        use_initial_guess = rf.check("use-initial-guess", Value(false)).asBool();

        nx={6,6};
        if(Bottle *nxB=rf.find("nx").asList())
        {
            if(nxB->size()!=nlines)
            {
                yError()<<"The number of parameters has to match the number of lines";
                yError()<<"nx="<<nxB->toString();
                return false;
            }
            nx[0]=nxB->get(0).asInt32();
            nx[1]=nxB->get(1).asInt32();
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
            ny[0]=nyB->get(0).asInt32();
            ny[1]=nyB->get(1).asInt32();
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
            marker_size[0]=mSzB->get(0).asFloat64();
            marker_size[1]=mSzB->get(1).asFloat64();
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
            marker_dist[0]=mDistB->get(0).asFloat64();
            marker_dist[1]=mDistB->get(1).asFloat64();
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
        lines_size.resize(nlines);
        for(int i=0; i<nlines;i++)
        {
            board[i]=cv::aruco::GridBoard::create(nx[i],ny[i],marker_size[i],marker_dist[i],dictionary[i]);
            lines_pose_world[i]=yarp::sig::Vector(7,0.0);
            lines_filter[i]=new iCub::ctrl::MedianFilter(line_filter_order,yarp::sig::Vector(7,0.0));
            lines_size[i]=yarp::sig::Vector(2,0.0);
            lines_size[i][0]=(marker_dist[i]+marker_size[i])*nx[i];
            lines_size[i][1]=marker_size[i]*ny[i];
        }

        rvec_v.resize(nlines);
        for(int i=0; i<nlines;i++)
        {
            rvec_v[i] = cv::Vec3d(0.0, 0.0, 0.0);
        }      
        tvec_v.resize(nlines);

        for(int i=0; i<nlines;i++)
        {
            tvec_v[i] = cv::Vec3d(0.0, 0.0, 0.0);
        }

        //configure the detector with corner refinement
        detector_params = cv::aruco::DetectorParameters::create();
        detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        std::string camera_remote = "/depthCamera";
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
                    fx=intrinsics->get(0).asFloat64();
                    fy=intrinsics->get(1).asFloat64();
                    px=intrinsics->get(2).asFloat64();
                    py=intrinsics->get(3).asFloat64();
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
            if (gCamera.check("remote"))
            {
                camera_remote = gCamera.find("remote").asString();
            }
        }

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
        
        if (!rgbdDrv.open(rgbdOpts)) {
            yError() << "Unable to talk to depthCamera!";
            return false;
        }

        simulation=rf.check("simulation",Value(false)).asBool();
        if (simulation)
        {
            gazeboPort.open("/" + getName() + "/gazebo:rpc");
        }
        imgInPort.open("/" + getName() + "/img:i");
        imgOutPort.open("/" + getName() + "/img:o");
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

        if (simulation && !updated_line_viewer)
        {
            if (viewerPort.getOutputCount()!=0)
            {
                if (update_line_sim("start-line") && update_line_sim("finish-line"))
                {
                    if (opcAdd("start-line") && opcAdd("finish-line"))
                    {
                        yarp::sig::Vector robot_pos;
                        get_model_pos("SIM_CER_ROBOT",robot_pos);
                        updated_line_viewer=create_line("start-line") &&
                                create_line("finish-line") &&
                                update_odometry(robot_pos[0],robot_pos[1],(180/M_PI)*robot_pos[6]);
                    }
                }
            }
        }

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
    bool get_model_pos(const std::string &model_name, yarp::sig::Vector &v)
    {
        Bottle cmd,rep;
        cmd.addString("getModelPos");
        cmd.addString(model_name);
        if (gazeboPort.write(cmd,rep))
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
    bool update_line_sim(const std::string &line_str)
    {
        int idx=line2idx[line_str];
        return get_model_pos(line_str,lines_pose_world[idx]);
    }

    /****************************************************************/
    int opcCheck(const std::string &line_tag)
    {
        int id=-1;
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab32("ask");

            Bottle &bLine=cmd.addList();
            Bottle &b1=bLine.addList();
            b1.addString(line_tag);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab32()==Vocab32::encode("ack"))
                {
                    if (Bottle *idB=rep.get(1).asList())
                    {
                        if (Bottle *idList=idB->get(1).asList())
                        {
                            if (idList->size()>0)
                            {
                                id=idList->get(0).asInt32();
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
            cmd.addVocab32("get");

            Bottle &l=cmd.addList();
            l.addString("id");
            l.addInt32(id);
            if (opcPort.write(cmd,rep))
            {
                if (rep.get(0).asVocab32()==Vocab32::encode("ack"))
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
    bool opcAdd(const std::string &line_tag)
    {
        int opc_id=opcCheck(line_tag);
        int i=line2idx[line_tag];
        return opcAdd(lines_pose_world[i],lines_size[i],opc_id,line_tag);
    }

    /****************************************************************/
    bool opcAdd(const yarp::sig::Vector &pose_world, const yarp::sig::Vector &line_size,
                const int opc_id, const std::string &line_tag)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab32("add");

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
                    if (rep.get(0).asVocab32()==Vocab32::encode("ack"))
                    {
                        int id=rep.get(1).asList()->get(1).asInt32();
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
            cmd.addVocab32("set");
            Bottle &pl=cmd.addList();
            pl.read(prop);
            Bottle id;
            Bottle &id_pl=id.addList();
            id_pl.addString("id");
            id_pl.addInt32(id_line);
            pl.append(id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab32()==Vocab32::encode("ack"));
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
            cmd.addVocab32("del");
            Bottle &pl=cmd.addList().addList();
            pl.addString("id");
            pl.addInt32(id);
            if (opcPort.write(cmd,rep))
            {
                if(rep.get(0).asVocab32()==Vocab32::encode("ack"))
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
        if(line2idx.count(line)==0)
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
        bool ret=create_line(line);
        if (line=="finish-line" && ret)
        {
            updated_line_viewer=true;
        }
        start_detection=false;

        return ret;
    }

    /****************************************************************/
    bool reset() override
    {
      std::lock_guard<std::mutex> lg(mtx_update);
    //   tvec.clear();
    //   rvec.clear();
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
    bool create_line(const std::string &line_tag)
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
        cmd.addFloat64(p0[0]);
        cmd.addFloat64(p0[1]);
        cmd.addFloat64(p0[2]);
        cmd.addFloat64(p1[0]);
        cmd.addFloat64(p1[1]);
        cmd.addFloat64(p1[2]);
        cmd.addInt32(color[0]);
        cmd.addInt32(color[1]);
        cmd.addInt32(color[2]);
        if(viewerPort.write(cmd,rep))
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
    void remove_from_viewer(const std::string &l)
    {
        Bottle cmd,rep;
        cmd.addString("delete_line");
        cmd.addString(l);
        if(viewerPort.write(cmd,rep))
        {
            if(rep.get(0).asVocab32()==Vocab32::encode("ok"))
            {
                yInfo()<<l<<"deleted";
                updated_line_viewer=false;
            }
        }
    }

    /****************************************************************/
    yarp::sig::Matrix toYarpMat(const cv::Mat &Rmat)
    {
        yarp::sig::Matrix R(Rmat.rows,Rmat.cols);
        for(int i=0;i<Rmat.rows;i++)
        {
            for(int j=0;j<Rmat.cols;j++)
            {
                R(i,j)=Rmat.at<double>(i,j);
            }
        }
        return R;
    }

    /****************************************************************/
    yarp::sig::Matrix update_world_frame(const std::string &line, const yarp::sig::Matrix &R,
                                         const yarp::sig::Vector &pose)
    {
        yarp::sig::Matrix M=eye(4);
        if (line=="start-line")
        {
            lineFrame.resize(4,4);
            lineFrame.setSubmatrix(R,0,0);
            lineFrame.setCol(3,pose);
            lineFrame.setRow(3,yarp::sig::Vector(3,0.0));
            yarp::sig::Matrix T=SE3inv(camFrame*lineFrame);
            M=T*camFrame;
        }
        if (line=="finish-line")
        {
            M=navFrame*camFrame;
        }
        return M;
    }

    /****************************************************************/
    void estimate_line(const yarp::sig::Matrix &wf, const yarp::sig::Vector &pose,
                       const int line_idx)
    {
        yarp::sig::Vector rot=yarp::math::dcm2axis(wf.submatrix(0,2,0,2));
        yarp::sig::Vector est_pose_world(7,0.0);
        est_pose_world.setSubvector(0,wf*pose);
        est_pose_world.setSubvector(3,rot);
        lines_pose_world[line_idx]=lines_filter[line_idx]->filt(est_pose_world);
        yarp::sig::Vector v=lines_pose_world[line_idx].subVector(3,5);
        if (norm(v)>0.0)
            v/=norm(v);
        lines_pose_world[line_idx].setSubvector(3,v);
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
            cv::aruco::refineDetectedMarkers(inImgMat,curr_board,corners,ids,rejected,cam_intrinsic,cam_distortion);
            cv::aruco::drawDetectedMarkers(inImgMat,corners,ids);

            //estimate pose
            //use cv::Mat and not cv::Vec3d to avoid issues with
            //cv::aruco::estimatePoseBoard() initial guess
            int valid=cv::aruco::estimatePoseBoard(corners,ids,curr_board,cam_intrinsic,cam_distortion,rvec_v[line_idx],tvec_v[line_idx], use_initial_guess);
            
            //if at least one board marker detected
            if (valid>0)
            {
                yInfo()<<"n. of valid markers:"<<valid;
                cv::aruco::drawAxis(inImgMat,cam_intrinsic,cam_distortion,rvec_v[line_idx],tvec_v[line_idx],0.5);
                 cv::Mat Rmat;
                cv::Rodrigues(rvec_v[line_idx],Rmat);
                yarp::sig::Matrix R=toYarpMat(Rmat);
                yarp::sig::Vector pose({tvec_v[line_idx][0],tvec_v[line_idx][1],tvec_v[line_idx][2],1.0});
                worldFrame=update_world_frame(line,R,pose);
                estimate_line(worldFrame,pose,line_idx);
                line_cnt++;
                if (line_cnt>line_filter_order)
                {
                    int opc_id=opcCheck(line);
                    opcAdd(lines_pose_world[line_idx],lines_size[line_idx],opc_id,line);
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
            Bottle *robotState=rep.get(0).asList();
            if (Bottle *loc=robotState->find("robot-location").asList())
            {
                yarp::sig::Vector robot_location(7,0.0);
                robot_location[0]=loc->get(0).asFloat64();
                robot_location[1]=loc->get(1).asFloat64();
                robot_location[5]=1.0;
                robot_location[6]=(M_PI/180)*loc->get(2).asFloat64();
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
        cmd.addFloat64(x);
        cmd.addFloat64(y);
        cmd.addFloat64(theta);
        if (navPort.write(cmd,rep))
        {
            if (rep.get(0).asVocab32()==Vocab32::encode("ok"))
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
        if (line2idx.count(line_tag)==0)
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
            cmd.addFloat64(pose[0]);
            cmd.addFloat64(pose[1]);
            cmd.addFloat64(theta);
            if (navPort.write(cmd,rep))
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
                    if (navPort.write(cmd,rep))
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

    bool set_initial_guess(bool flag) override {
        std::lock_guard<std::mutex> lg(mtx_update);
        use_initial_guess = flag;

        return true;
    }

    bool get_initial_guess() override {
        std::lock_guard<std::mutex> lg(mtx_update);
        return use_initial_guess;
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
        if (simulation)
        {
            gazeboPort.interrupt();
        }
        imgInPort.interrupt();
        imgOutPort.interrupt();
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

        rgbdDrv.close();

        if (simulation)
        {
            gazeboPort.close();
        }

        imgInPort.close();
        imgOutPort.close();
        opcPort.close();
        gazeStatePort.close();
        navPort.close();
        viewerPort.close();
        cmdPort.close();
        return true;
    }

    /****************************************************************/
    bool getCameraOptions(cv::Mat &cam_intrinsics, cv::Mat &cam_distortion)
    {
        yarp::dev::IRGBDSensor* irgbd;
        rgbdDrv.view(irgbd);

        if (irgbd != nullptr) {
            yInfo() << "Reading intrinsics from camera";
            yarp::os::Property intrinsics;
            if (irgbd->getRgbIntrinsicParam(intrinsics)) {
                fx = intrinsics.find("focalLengthX").asFloat64();
                fy = intrinsics.find("focalLengthY").asFloat64();
                px = intrinsics.find("principalPointX").asFloat64();
                py = intrinsics.find("principalPointY").asFloat64();
                
                yInfo() << "Camera intrinsics:" << fx << fy << px << py;
                
                cam_intrinsics.at<double>(0, 0) = fx;
                cam_intrinsics.at<double>(0, 2) = px;
                cam_intrinsics.at<double>(1, 1) = fy;
                cam_intrinsics.at<double>(1, 2) = py;
                cam_intrinsics.at<double>(2, 2) = 1.0;

                cam_distortion.at<double>(0, 0) = 0.0; //k1;
                cam_distortion.at<double>(0, 1) = 0.0; //k2;
                cam_distortion.at<double>(0, 2) = 0.0; //t1;
                cam_distortion.at<double>(0, 3) = 0.0; //t2;
                cam_distortion.at<double>(0, 4) = 0.0; //k3;
                
                rgbdDrv.close();
                return true;
            }
        }

        return false;
    }

public:

    /****************************************************************/
    Detector() : start_detection(false), updated_cam(false), updated_nav(false),
        updated_odom(false), camera_configured(false), updated_line_viewer(false), 
        lineFrame(eye(4)), camFrame(eye(4)), navFrame(eye(4)), worldFrame(eye(4))  { }

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
