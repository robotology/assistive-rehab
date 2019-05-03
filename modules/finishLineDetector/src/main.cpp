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
    cv::Mat rvec,tvec; // Rodrigues coefficients wrt cam
    yarp::sig::Vector est_pose;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle> posePort;
    yarp::os::RpcClient camPort;

    /****************************************************************/
    bool configure(ResourceFinder &rf) override
    {
        //parameters
        std::string moduleName = rf.check("name", Value("finishLineDetector")).asString();
        setName(moduleName.c_str());

        period = rf.check("period",Value(0.1)).asDouble();
        nx = rf.check("nx",Value(4)).asInt();
        ny = rf.check("ny",Value(1)).asInt();
        marker_size = rf.check("marker-size",Value(0.17)).asDouble();
        marker_dist = rf.check("marker-dist",Value(0.05)).asDouble();;

        //create dictionary
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_50);
        board = cv::aruco::GridBoard::create(nx,ny,marker_size,marker_dist,dictionary);

        //configure the detector with corner refinement
        detector_params = cv::aruco::DetectorParameters::create();
        detector_params->doCornerRefinement = true;

        //final estimate
        est_pose.resize(7);

        imgInPort.open("/" + getName() + "/img:i");
        imgOutPort.open("/" + getName() + "/img:o");
        posePort.open("/" + getName() + "/pose:o");
        camPort.open("/" + getName() + "/cam:rpc");

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
        if (yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImg=imgInPort.read())
        {
            yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImg=imgOutPort.prepare();
            cv::Mat inImgMat;
            inImgMat=yarp::cv::toCvMat(*inImg);

            //detect markers
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f> > corners,rejected;
            cv::aruco::detectMarkers(inImgMat,dictionary,corners,ids);

            //read camera params
            getCameraOptions(cam_intrinsic,cam_distortion);

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
                                                       rvec,tvec);

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
                    est_pose[0]=tvec.at<double>(0);
                    est_pose[1]=tvec.at<double>(1);
                    est_pose[2]=tvec.at<double>(2);
                    est_pose[3]=rot[0];
                    est_pose[4]=rot[1];
                    est_pose[5]=rot[2];
                    est_pose[6]=rot[3];

                    yarp::os::Bottle &pose=posePort.prepare();
                    pose.clear();
                    pose.addList().read(est_pose);
                    posePort.write();
                }
            }
            outImg=yarp::cv::fromCvMat<yarp::sig::PixelRgb>(inImgMat);
            imgOutPort.write();
        }
        return true;
    }

    /****************************************************************/
    bool interruptModule() override
    {
        imgInPort.interrupt();
        imgOutPort.interrupt();
        posePort.interrupt();
        camPort.interrupt();

        return true;
    }

    /****************************************************************/
    bool close() override
    {
        imgInPort.close();
        imgOutPort.close();
        posePort.close();
        camPort.close();

        return true;
    }

    /****************************************************************/
    bool getCameraOptions(cv::Mat &cam_intrinsic, cv::Mat &cam_distortion)
    {
        if (camPort.getOutputCount()>0)
        {
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
                    yarp::os::Bottle *principal_x = intrinsics->get(6).asList();
                    yarp::os::Bottle *principal_y = intrinsics->get(7).asList();
                    yarp::os::Bottle *tangential_1 = intrinsics->get(9).asList();
                    yarp::os::Bottle *tangential_2 = intrinsics->get(10).asList();

                    double fx = focal_x->get(1).asDouble();
                    double fy = focal_y->get(1).asDouble();
                    double k1 = radial_1->get(1).asDouble();
                    double k2 = radial_2->get(1).asDouble();
                    double k3 = radial_3->get(1).asDouble();
                    double t1 = tangential_1->get(1).asDouble();
                    double t2 = tangential_2->get(1).asDouble();
                    double px = principal_x->get(1).asDouble();
                    double py = principal_y->get(1).asDouble();

                    cam_intrinsic = cv::Mat::eye(3,3,CV_64F);
                    cam_distortion = cv::Mat::zeros(1,5,CV_64F);
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
