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
    int nx,ny;
    float marker_size,marker_dist;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::GridBoard> board;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params;
    cv::Mat cam_intrinsic,cam_distortion;
    cv::Vec3d rvec,tvec; // Rodrigues coefficients wrt cam
    yarp::sig::Matrix gaze_frame;
    int opc_id;
    bool add;
    bool camera_configured;
    double fx,fy,px,py;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgOutPort;
    yarp::os::RpcClient camPort;
    yarp::os::RpcClient opcPort;
    yarp::os::BufferedPort<yarp::os::Property> gazeStatePort;

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        //parameters
        std::string moduleName = rf.check("name", Value("finishLineDetector")).asString();
        setName(moduleName.c_str());

        period = rf.check("period",Value(0.1)).asDouble();
        nx = rf.check("nx",Value(6)).asInt();
        ny = rf.check("ny",Value(1)).asInt();
        marker_size = rf.check("marker-size",Value(0.13)).asDouble();
        marker_dist = rf.check("marker-dist",Value(0.005)).asDouble();

        //create dictionary
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);
        board = cv::aruco::GridBoard::create(nx,ny,marker_size,marker_dist,dictionary);

        //configure the detector with corner refinement
        detector_params = cv::aruco::DetectorParameters::create();
        detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        add=true;
        opc_id=-1;
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

            if (yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImg=imgInPort.read())
            {
                yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImg=imgOutPort.prepare();
                cv::Mat inImgMat;
                inImgMat=yarp::cv::toCvMat(*inImg);

                //detect markers
                std::vector<int> ids;
                std::vector<std::vector<cv::Point2f> > corners,rejected;
                cv::aruco::detectMarkers(inImgMat,dictionary,corners,ids);

                //configure camera
                if(!camera_configured)
                {
                    camera_configured=getCameraOptions(cam_intrinsic,cam_distortion);
                }

                //perform refinement
                cv::aruco::refineDetectedMarkers(inImgMat,board,corners,ids,rejected,
                                                 cam_intrinsic,cam_distortion);

                //if at least one marker detected
                if(ids.size()>0)
                {
                    cv::aruco::drawDetectedMarkers(inImgMat,corners,ids);

                    //estimate pose
                    //use cv::Mat and not cv::Vec3d to avoid issues with
                    //cv::aruco::estimatePoseBoard() initial guess
                    int valid=cv::aruco::estimatePoseBoard(corners,ids,board,
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

                        yInfo() << "Pose wrt camera" << est_pose_camera.toString();
                        yInfo() << "Pose wrt root" << est_pose_root.toString();

                        opcAdd(est_pose_camera,est_pose_root,add);
                        if(add && opc_id!=-1)
                        {
                            add=false;
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
                    if(opc_id!=-1)
                    {
                        if (opcPort.getOutputCount())
                        {
                            Bottle cmd,rep;
                            cmd.addVocab(Vocab::encode("get"));
                            Bottle &content=cmd.addList().addList();
                            content.addString("id");
                            content.addInt(opc_id);
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
                                                            prop_line.put("finish-line",line.get(0));
                                                            opcSet(prop_line);
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
                const bool &add)
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

            poseProp.put("visibility",true);

            Property prop;
            Bottle line;
            line.addList().read(poseProp);
            prop.put("finish-line",line.get(0));

            if(add)
            {
                cmd.addList().read(prop);
                if (opcPort.write(cmd,rep))
                {
                    if (rep.get(0).asVocab()==Vocab::encode("ack"))
                    {
                        opc_id=rep.get(1).asList()->get(1).asInt();
                        return opcSet(prop);
                    }
                }
            }
            else
            {
                return opcSet(prop);
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcSet(const yarp::os::Property &prop)
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
            id_pl.addInt(opc_id);
            pl.append(id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab()==Vocab::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    bool opcDel()
    {
        if (opcPort.getOutputCount())
        {
            Bottle cmd,rep;
            cmd.addVocab(Vocab::encode("del"));
            Bottle &pl=cmd.addList().addList();
            pl.addString("id");
            pl.addInt(opc_id);
            if (opcPort.write(cmd,rep))
            {
                return (rep.get(0).asVocab()==Vocab::encode("ack"));
            }
        }

        return false;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        opcDel();

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
