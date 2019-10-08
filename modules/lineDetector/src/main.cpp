#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Image.h>
#include <yarp/cv/Cv.h>
#include <yarp/dev/IVisualParams.h>
#include <yarp/dev/GenericVocabs.h>
#include <yarp/math/Math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace yarp::os;

class Detector : public RFModule
{
    double period;
    std::vector< cv::Ptr<cv::aruco::Dictionary> > dictionary;
    std::vector< cv::Ptr<cv::aruco::GridBoard> > board;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
    cv::Mat cam_intrinsic,cam_distortion;
    yarp::sig::Matrix gaze_frame;
    std::vector<int> opc_id;
    std::vector<bool> add;
    bool camera_configured;
    double fx,fy,px,py;
    cv::Vec3d rvec,tvec; // Rodrigues coefficients wrt cam

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgOutPort;
    yarp::os::RpcClient camPort;
    yarp::os::RpcClient opcPort;
    yarp::os::BufferedPort<yarp::os::Property> gazeStatePort;

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        //parameters
        std::string moduleName = rf.check("name", Value("lineDetector")).asString();
        setName(moduleName.c_str());

        period = rf.check("period",Value(0.1)).asDouble();

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
        if (Property *p=gazeStatePort.read(false))
        {
            yarp::sig::Vector pose;
            p->find("depth_rgb").asList()->write(pose);
            gaze_frame=yarp::math::axis2dcm(pose.subVector(3,6));
            gaze_frame.setSubcol(pose.subVector(0,2),0,3);

            //configure camera
            if(!camera_configured)
            {
                camera_configured=getCameraOptions(cam_intrinsic,cam_distortion);
                return true;
            }

            if (yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImg=imgInPort.read())
            {
                yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImg=imgOutPort.prepare();
                cv::Mat inImgMat;
                inImgMat=yarp::cv::toCvMat(*inImg);

                //detect markers
                std::vector< std::vector<int> > ids(2);
                std::vector< std::vector<std::vector<cv::Point2f> > > corners(2);
                cv::aruco::detectMarkers(inImgMat,dictionary[0],corners[0],ids[0]);
                cv::aruco::detectMarkers(inImgMat,dictionary[1],corners[1],ids[1]);

                //if at least one marker detected in one line
                for(int line_idx=0;line_idx<ids.size();line_idx++)
                {
                    if(ids[line_idx].size()>0)
                    {
                        std::vector<int> curr_ids=ids[line_idx];
                        std::vector<std::vector<cv::Point2f> > curr_corners=corners[line_idx];
                        std::vector<std::vector<cv::Point2f> > rejected;
                        cv::Ptr<cv::aruco::GridBoard> curr_board=board[line_idx];

                        //perform refinement
                        cv::aruco::refineDetectedMarkers(inImgMat,curr_board,curr_corners,curr_ids,rejected,
                                                         cam_intrinsic,cam_distortion);

                        cv::aruco::drawDetectedMarkers(inImgMat,curr_corners,curr_ids);

                        //estimate pose
                        //use cv::Mat and not cv::Vec3d to avoid issues with
                        //cv::aruco::estimatePoseBoard() initial guess
                        int valid=cv::aruco::estimatePoseBoard(curr_corners,curr_ids,curr_board,
                                                               cam_intrinsic,cam_distortion,
                                                               rvec,tvec,true);

                        //if at least one board marker detected
                        if(valid>0)
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
                            yarp::sig::Vector rot=yarp::math::dcm2axis(R);
                            yarp::sig::Vector est_pose_camera(7),est_pose_root(7);
                            est_pose_camera[0]=tvec[0];
                            est_pose_camera[1]=tvec[1];
                            est_pose_camera[2]=tvec[2];
                            est_pose_camera[3]=rot[0];
                            est_pose_camera[4]=rot[1];
                            est_pose_camera[5]=rot[2];
                            est_pose_camera[6]=rot[3];

                            yarp::sig::Vector pose=est_pose_camera.subVector(0,2);
                            pose.push_back(1.0);
                            est_pose_root.setSubvector(0,gaze_frame*pose);
                            yarp::sig::Vector rot_root=yarp::math::dcm2axis(gaze_frame.submatrix(0,2,0,2)*R.submatrix(0,2,0,2));
                            est_pose_root[3]=rot_root[0];
                            est_pose_root[4]=rot_root[1];
                            est_pose_root[5]=rot_root[2];
                            est_pose_root[6]=rot_root[3];

                            yInfo() << "Line" << line_idx << "pose wrt camera" << est_pose_camera.toString();
                            yInfo() << "Line" << line_idx << "pose wrt root" << est_pose_root.toString();

                            bool add_to_opc=add[line_idx];
                            opcAdd(est_pose_camera,est_pose_root,add_to_opc,line_idx,true);
                            if(add_to_opc && opc_id[line_idx]!=-1)
                            {
                                add[line_idx]=false;
                            }
                        }
                        else
                        {
                            yInfo()<<"No board marker detected";
                        }
                    }
                    else
                    {
                        yInfo()<<"No marker detected";
                        if(opc_id[line_idx]!=-1)
                        {
                            if (opcPort.getOutputCount())
                            {
                                Bottle cmd,rep;
                                cmd.addVocab(Vocab::encode("get"));
                                Bottle &content=cmd.addList().addList();
                                content.addString("id");
                                content.addInt(opc_id[line_idx]);
                                if(opcPort.write(cmd,rep))
                                {
                                    if(rep.get(0).asVocab()==Vocab::encode("ack"))
                                    {
                                        if(Bottle *prop1=rep.get(1).asList())
                                        {
                                            if(Bottle *prop2=prop1->get(0).asList())
                                            {
                                                if(Bottle *prop3=prop2->get(1).asList())
                                                {
                                                    Property prop(prop3->toString().c_str());
                                                    if(prop.check("pose_camera") && prop.check("pose_root"))
                                                    {
                                                        if(Bottle *bPoseCam=prop.find("pose_camera").asList())
                                                        {
                                                            if(Bottle *bPoseRoot=prop.find("pose_root").asList())
                                                            {
                                                                Property poseProp;
                                                                Bottle pc; pc.addList().read(*bPoseCam);
                                                                Bottle pr; pr.addList().read(*bPoseRoot);
                                                                poseProp.put("pose_camera",pc.get(0));
                                                                poseProp.put("pose_root",pr.get(0));
                                                                poseProp.put("visibility",false);

                                                                Property prop_line;
                                                                Bottle line;
                                                                line.addList().read(poseProp);
                                                                if(line_idx==0)
                                                                {
                                                                    prop_line.put("start-line",line.get(0));
                                                                }
                                                                else if(line_idx==1)
                                                                {
                                                                    prop_line.put("finish-line",line.get(0));
                                                                }
                                                                opcSet(prop_line,opc_id[line_idx]);
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                outImg=yarp::cv::fromCvMat<yarp::sig::PixelRgb>(inImgMat);
                imgOutPort.write();
            }
        }
        return true;
    }

    /****************************************************************/
    bool opcAdd(const yarp::sig::Vector &pose_camera, const yarp::sig::Vector &pose_root,
                const bool &add, const int idx, const bool &visible)
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("add"));

            Bottle bPoseCam;
            Property poseProp;
            bPoseCam.addList().read(pose_camera);
            poseProp.put("pose_camera",bPoseCam.get(0));

            Bottle bPoseRoot;
            bPoseRoot.addList().read(pose_root);
            poseProp.put("pose_root",bPoseRoot.get(0));

            poseProp.put("visibility",visible);

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
                    yInfo()<<"Line"<<id<<"removed from opc";
                    return true;
                }
            }
        }

        return false;
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
        return true;
    }

    /****************************************************************/
    bool close() override
    {
        imgInPort.close();
        imgOutPort.close();
        camPort.close();
        opcPort.close();
        gazeStatePort.close();
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
