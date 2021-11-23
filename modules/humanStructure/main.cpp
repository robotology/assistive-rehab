/*
 * Copyright (C) 2019 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/cv/Cv.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RpcClient.h>
#include <yarp/math/Math.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <cstring>
#include <vector>
#include <iostream>
#include <utility>
#include <cmath>

#include "humanStructure_IDLServer.h"

using namespace yarp::math;

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::os::Bottle>
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageOutPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >   imageOutDepthPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >   imageOutSegmentPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat>>   imageInFloat;
    yarp::os::BufferedPort<yarp::os::Bottle >    targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle >    blobPort;
    yarp::os::RpcClient camPort;
    yarp::os::RpcClient armPort;

    int followSkeletonIndex;
    cv::Mat overlayFrame;
    cv::Mat overlayObject;

    bool    camera_configured;
    double  fov_h;
    double  fov_v;

    double minVal;
    double maxVal;
    
    bool isArmLifted;
    

    yarp::sig::ImageOf<yarp::sig::PixelFloat> depth;

public:
    int armthresh;
    /********************************************************/

    Processing( const std::string &moduleName )
    {
        this->moduleName = moduleName;
    }

    /********************************************************/
    ~Processing()
    {
    };

    /********************************************************/
    bool setDepthValues(const double min, const double max ){

        minVal = min;
        maxVal = max;

        yDebug() << "setting Min val to " << minVal << "setting max val to " << maxVal;

        return true;
    }

    /********************************************************/
    bool open()
    {
        this->useCallback();

        BufferedPort<yarp::os::Bottle >::open( "/" + moduleName + "/skeleton:i" );
        imageInPort.open("/" + moduleName + "/image:i");

        imageOutPort.open("/" + moduleName + "/image:o");
        imageOutDepthPort.open("/" + moduleName + "/depth:o");
        imageOutSegmentPort.open("/" + moduleName + "/segmented:o");

        targetPort.open("/" + moduleName + "/target:o");
        blobPort.open("/" + moduleName + "/blobs:o");
        armPort.open("/" + moduleName + "/trigger:o");

        imageInFloat.open("/" + moduleName + "/float:i");

        camPort.open("/" + moduleName + "/cam:rpc");

        followSkeletonIndex = -1;

        isArmLifted = false;
        
        armthresh = 20;

        return true;
    }

    /********************************************************/
    void close()
    {
        imageInFloat.close();
        imageOutPort.close();
        imageInPort.close();
        BufferedPort<yarp::os::Bottle >::close();
        targetPort.close();
        blobPort.close();
        camPort.close();
        imageOutDepthPort.close();
        imageOutSegmentPort.close();
        armPort.close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle >::interrupt();
    }

    /**********************************************************/
    int findArmLift( const yarp::os::Bottle &blobs,
                    const yarp::os::Bottle &skeletons)
    {
        int index = -1;

        if (skeletons.size() > 0)
        {
            size_t skeletonSize = skeletons.get(0).asList()->size();
            size_t internalElements = 0;

            yDebug() << "skeleton " << skeletonSize;

            std::vector<cv::Point> relbow;
            std::vector<cv::Point> rwrist;
            std::vector<cv::Point> lelbow;
            std::vector<cv::Point> lwrist;
            std::vector<cv::Point> neck;
            std::vector<cv::Point> midHip;

            cv::Point point;

            if (skeletonSize>0)
                internalElements = skeletons.get(0).asList()->get(0).asList()->size();

            for (size_t i = 0; i < skeletonSize; i++)
            {
                if (yarp::os::Bottle *propField = skeletons.get(0).asList()->get(i).asList())
                {
                    for (int ii = 0; ii < internalElements; ii++)
                    {
                        if (yarp::os::Bottle *propFieldPos = propField->get(ii).asList())
                        {
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RElbow") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asFloat64();
                                point.y = (int)propFieldPos->get(2).asFloat64();
                                relbow.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RWrist") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asFloat64();
                                point.y = (int)propFieldPos->get(2).asFloat64();
                                rwrist.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LElbow") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asFloat64();
                                point.y = (int)propFieldPos->get(2).asFloat64();
                                lelbow.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LWrist") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asFloat64();
                                point.y = (int)propFieldPos->get(2).asFloat64();
                                lwrist.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"Neck") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asFloat64();
                                point.y = (int)propFieldPos->get(2).asFloat64();
                                neck.push_back(point);
                            }
                            if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"MidHip") == 0)
                            {
                                point.x = (int)propFieldPos->get(1).asFloat64();
                                point.y = (int)propFieldPos->get(2).asFloat64();
                                midHip.push_back(point);
                            }
                        }
                    }
                }
            }

            int getIndex = -1;

            if (neck.size() > 0)
            {
                for (int i = 0; i < skeletonSize; i++)
                {
                    if ( relbow[i].y - rwrist[i].y > armthresh && relbow[i].y > 0 && rwrist[i].y > 0 )
                    {
                        getIndex = i;
                    }
                    else if( lelbow[i].y - lwrist[i].y > armthresh && lelbow[i].y > 0 && lwrist[i].y > 0 )
                    {
                        getIndex = i;
                    }
                }
                if (getIndex > -1)
                {
                    yError() << "************************" << blobs.size();
                    for (int i=0;i<blobs.size(); i++)
                    {
                        yarp::os::Bottle *item=blobs.get(i).asList();

                        int cog = item->get(0).asInt32();//item->get(2).asInt32() - ( (item->get(2).asInt32() -item->get(0).asInt32()) / 2);

                        if ( abs(cog - neck[getIndex].x) < armthresh)
                            index = i;
                    }
                }
            }
        }
        return index;
    }

    /********************************************************/
    bool getCameraOptions()
    {   fov_h = 55;
        fov_v = 42;
        /*if (camPort.getOutputCount()>0)
        {
            yarp::os::Bottle cmd,rep;
            cmd.addVocab32("visr");
            cmd.addVocab32("get");
            cmd.addVocab32("fov");
            if (camPort.write(cmd,rep))
            {
                if (rep.size()>=5)
                {
                    fov_h=rep.get(3).asFloat64();
                    fov_v=rep.get(4).asFloat64();
                    yInfo()<<"camera fov_h (from sensor) ="<<fov_h;
                    yInfo()<<"camera fov_v (from sensor) ="<<fov_v;
                    return true;
                }
            }
        }*/

        return true;
    }

    /********************************************************/
    bool getPoint3D(const int u, const int v, yarp::sig::Vector &p) const
    {
        if ((u>=0) && (u<depth.width()) && (v>=0) && (v<depth.height()))
        {
            double f=depth.width()/(2.0*tan(fov_h*(M_PI/180.0)/2.0));
            double d=depth(u,v);
            if ((d>0.0) && (f>0.0))
            {
                double x=u-0.5*(depth.width()-1);
                double y=v-0.5*(depth.height()-1);

                p=d*ones(3);
                p[0]*=x/f;
                p[1]*=y/f;

                return true;
            }
        }
        return false;
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &data )
    {
        if (!camera_configured)
            camera_configured=getCameraOptions();

        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage = imageOutPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outDepth = imageOutDepthPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelMono> &outSegment = imageOutSegmentPort.prepare();


        while (imageInPort.getInputCount() < 1 || imageInFloat.getInputCount() < 1 )
        {
            yError() << "waiting for connections";
            yarp::os::Time::delay(0.1);
        }
        yDebug()<< "setting up system";


        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = imageInPort.read();
        yDebug()<< __LINE__;
        yarp::sig::ImageOf<yarp::sig::PixelFloat> *float_yarp = imageInFloat.read();

        yDebug()<< __LINE__;
        depth = *float_yarp;
        cv::Mat float_cv = yarp::cv::toCvMat(*float_yarp);
        cv::Mat mono_cv = cv::Mat::ones(float_yarp->height(), float_yarp->width(), CV_8UC1);
        float_cv -= minVal;
        float_cv.convertTo(mono_cv, CV_8U, 255.0/(maxVal-minVal) );
        cv::Mat depth_cv(float_yarp->height(), float_yarp->width(), CV_8UC1, cv::Scalar(255));
        cv::Mat segment_cv(inImage->height(), inImage->width(), CV_8UC1, cv::Scalar(0));
        depth_cv = depth_cv - mono_cv;
        cv::Mat mask;
        inRange(depth_cv, cv::Scalar(255), cv::Scalar(255), mask);
        cv::Mat black_image(depth_cv.size(), CV_8U, cv::Scalar(0));
        black_image.copyTo(depth_cv, mask);
        yarp::os::Stamp stamp;
        imageInPort.getEnvelope(stamp);
        yarp::os::Bottle &target  = targetPort.prepare();
        size_t skeletonSize = data.get(0).asList()->size();
        size_t internalElements = 0;

        if (skeletonSize>0)
        {
            target = data;
            internalElements = data.get(0).asList()->get(0).asList()->size();
        }

        outImage = *inImage;

        outImage.resize(inImage->width(), inImage->height());

        std::vector<cv::Point> rightEye2D;
        std::vector<cv::Point> leftEye2D;
        std::vector<cv::Point> neck2D;
        std::vector<cv::Point> nose2D;
        std::vector<cv::Point> rightEar2D;
        std::vector<cv::Point> leftEar2D;
        std::vector<cv::Point> leftShoulder2D;
        std::vector<cv::Point> rightShoulder2D;
        std::vector<cv::Point> rightWrist2D;
        std::vector<cv::Point> leftWrist2D;
        std::vector<cv::Point> rightElbow2D;
        std::vector<cv::Point> leftElbow2D;
        std::vector<cv::Point> MidHip2D;
        std::vector<cv::Point> LHip2D;
        std::vector<cv::Point> RHip2D;
        cv::Point point;

        std::vector<yarp::os::Bottle> shapes;
        shapes.clear();
        std::vector<std::pair <int,int> > elements;
        elements.clear();

        yDebug() << "skeletonSize" << skeletonSize;

        for (int i = 0; i < skeletonSize; i++)
        {
            if (yarp::os::Bottle *propField = data.get(0).asList()->get(i).asList())
            {
                for (int ii = 0; ii < internalElements; ii++)
                {
                    if (yarp::os::Bottle *propFieldPos = propField->get(ii).asList())
                    {
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"REye") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            rightEye2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LEye") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            leftEye2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"REar") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            rightEar2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LEar") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            leftEar2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"Neck") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            neck2D.push_back(point);

                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"Nose") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            nose2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LShoulder") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            leftShoulder2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RShoulder") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            rightShoulder2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RWrist") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            rightWrist2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LWrist") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            leftWrist2D.push_back(point);
                        }

                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RElbow") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            rightElbow2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LElbow") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            leftElbow2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"MidHip") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            MidHip2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LHip") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            LHip2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RHip") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asFloat64();
                            point.y = (int)propFieldPos->get(2).asFloat64();
                            RHip2D.push_back(point);
                        }
                    }
                }
            }
        }

        cv::Mat out_cv = yarp::cv::toCvMat(outImage);

        //need this increment as case might be that skeleton does not
        //satisfy conditions to fill in bottle
        int increment = 0;

        for (size_t i = 0; i < skeletonSize; i++)
        {
            cv::Point topLeft;
            cv::Point bottomRight;

            yInfo() << "###########";
            yInfo() << "MidHip2D " << MidHip2D[i].x << MidHip2D[i].y << "NecK2D " << neck2D[i].x << neck2D[i].y;
            yarp::os::Bottle tmp;
            tmp.clear();
            if (MidHip2D[i].x > 0)
            {
                yInfo() << "in midHip";
                tmp.addInt32(MidHip2D[i].x);
                tmp.addInt32(MidHip2D[i].y);

                elements.push_back(std::make_pair(MidHip2D[i].x,increment));
                shapes.push_back(tmp);
                increment++;
            }
            else if (neck2D[i].x > 0)
            {
                yInfo() << "in neck";
                tmp.addInt32(neck2D[i].x);
                tmp.addInt32(neck2D[i].y);
                elements.push_back(std::make_pair(neck2D[i].x,increment));
                shapes.push_back(tmp);
                increment++;
            }
            else
            {
                yError() << "Cannot compute...cannot detect anything";
            }
        }

        yarp::os::Bottle list;
        yInfo() << "**************** Skeleton Size" << skeletonSize << "Element SIZE " << elements.size() << "shape SIZE"  << shapes.size() ;
        if (elements.size()>0)
        {
            for (int i=0; i<elements.size(); i++)
                yInfo() << "Testing elements " << i << elements[i].first << elements[i].second;

            std::sort(elements.begin(), elements.end());

            for (int i=0; i<elements.size(); i++)
                yInfo() << "Sorted elements " << i << elements[i].first << elements[i].second;

            if ( shapes.size() > 0)
            {
                yarp::os::Bottle blobs;
                blobs.clear();

                for (int i=0; i<shapes.size(); i++)
                {
                    yInfo() << "**************** shapes elements" << shapes[elements[i].second].toString();
                    yarp::os::Bottle &tmp = blobs.addList();
                    tmp.addInt32(shapes[elements[i].second].get(0).asInt32());
                    tmp.addInt32(shapes[elements[i].second].get(1).asInt32());
                }

                list = blobs;
                targetPort.setEnvelope(stamp);
                targetPort.write();
            }
        }

        if ( shapes.size() > 0)
        {
            int index = findArmLift(list, data);
            out_cv.copyTo(overlayFrame);
            out_cv.copyTo(overlayObject);

            if (index > -1 )
                followSkeletonIndex = index;

            yDebug() << " Looking at skeleton " << followSkeletonIndex;

            int finalIndex = -1;
            for (int i=0;i<list.size(); i++)
            {
                yarp::os::Bottle *item=list.get(i).asList();

                int cog = item->get(0).asInt32(); //item->get(2).asInt32() - ( (item->get(2).asInt32() -item->get(0).asInt32()) / 2);

                if ( abs(cog - neck2D[followSkeletonIndex].x) < 30)
                    finalIndex = i;
            }

            if (followSkeletonIndex > -1 && finalIndex > -1)
            {
                yDebug() << "Final index is " << finalIndex;

                if (leftEye2D[finalIndex].x > 0 && leftEye2D[finalIndex].y > 0 && leftEye2D[finalIndex].x > 0 && leftEye2D[finalIndex].y > 0)
                    line(overlayFrame, leftEye2D[finalIndex], nose2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (rightEye2D[finalIndex].x > 0 && rightEye2D[finalIndex].y > 0 && nose2D[finalIndex].x > 0 && nose2D[finalIndex].y > 0)
                    line(overlayFrame, rightEye2D[finalIndex], nose2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (leftEar2D[finalIndex].x > 0 && leftEar2D[finalIndex].y > 0 && leftEye2D[finalIndex].x > 0 && leftEye2D[finalIndex].y > 0)
                    line(overlayFrame, leftEye2D[finalIndex], leftEar2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (rightEar2D[finalIndex].x > 0 && rightEar2D[finalIndex].y > 0 && rightEye2D[finalIndex].x > 0 && rightEye2D[finalIndex].y > 0)
                    line(overlayFrame, rightEye2D[finalIndex], rightEar2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (nose2D[finalIndex].x > 0 && nose2D[finalIndex].y > 0 && neck2D[finalIndex].x > 0 && neck2D[finalIndex].y > 0)
                    line(overlayFrame, nose2D[finalIndex], neck2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (neck2D[finalIndex].x > 0 && neck2D[finalIndex].y > 0 && leftShoulder2D[finalIndex].x > 0 && leftShoulder2D[finalIndex].y > 0)
                    line(overlayFrame, neck2D[finalIndex], leftShoulder2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (leftElbow2D[finalIndex].x > 0 && leftElbow2D[finalIndex].y > 0 && leftShoulder2D[finalIndex].x > 0 && leftShoulder2D[finalIndex].y > 0)
                    line(overlayFrame, leftShoulder2D[finalIndex], leftElbow2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (leftWrist2D[finalIndex].x > 0 && leftWrist2D[finalIndex].y > 0 && leftElbow2D[finalIndex].x > 0 && leftElbow2D[finalIndex].y > 0)
                    line(overlayFrame, leftElbow2D[finalIndex], leftWrist2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (neck2D[finalIndex].x > 0 && neck2D[finalIndex].y > 0 && rightShoulder2D[finalIndex].x > 0 && rightShoulder2D[finalIndex].y > 0)
                    line(overlayFrame, neck2D[finalIndex], rightShoulder2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (rightElbow2D[finalIndex].x > 0 && rightElbow2D[finalIndex].y > 0 && rightShoulder2D[finalIndex].x > 0 && rightShoulder2D[finalIndex].y > 0)
                    line(overlayFrame, rightShoulder2D[finalIndex], rightElbow2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (rightWrist2D[finalIndex].x > 0 && rightWrist2D[finalIndex].y > 0 && rightElbow2D[finalIndex].x > 0 && rightElbow2D[finalIndex].y > 0)
                    line(overlayFrame, rightElbow2D[finalIndex], rightWrist2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);

                if (RHip2D[finalIndex].x > 0 && LHip2D[finalIndex].x > 0 && neck2D[finalIndex].x > 0 && neck2D[finalIndex].y > 0)
                {
                    line(overlayFrame, neck2D[finalIndex], LHip2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);
                    line(overlayFrame, neck2D[finalIndex], RHip2D[finalIndex], cv::Scalar(118, 185 , 0), 4, 8);
                }

                double opacity = 0.4;
                cv::addWeighted(overlayFrame, opacity, out_cv, 1 - opacity, 0, out_cv);

                yDebug() << "neck 2D " << neck2D[finalIndex].x << neck2D[finalIndex].y;
                yDebug() << "left wrist 2D " << leftWrist2D[finalIndex].x << leftWrist2D[finalIndex].y;
                yDebug() << "right wrist 2D " << rightWrist2D[finalIndex].x << rightWrist2D[finalIndex].y;
                yDebug() << "left elbow 2D " << leftElbow2D[finalIndex].x << leftElbow2D[finalIndex].y;
                yDebug() << "right elbow 2D " << rightElbow2D[finalIndex].x << rightElbow2D[finalIndex].y;
                yDebug() << "right hip 2D " << RHip2D[finalIndex].x << RHip2D[finalIndex].y;
                yDebug() << "left hip 2D " << LHip2D[finalIndex].x << LHip2D[finalIndex].y;
                //yDebug() << "mid hip 2D " << MidHip2D[finalIndex].x << MidHip2D[finalIndex].y;

                std::map< double, cv::Point> valueVector;

                double noseValue = depth_cv.at<uchar>(nose2D[finalIndex]);
                double neckValue = depth_cv.at<uchar>(neck2D[finalIndex]);
                double leftWristValue = depth_cv.at<uchar>(leftWrist2D[finalIndex]);
                double rightWristValue = depth_cv.at<uchar>(rightWrist2D[finalIndex]);
                double leftElbowValue = depth_cv.at<uchar>(leftElbow2D[finalIndex]);
                double rightElbowValue = depth_cv.at<uchar>(rightElbow2D[finalIndex]);
                //double midHipValue = depth_cv.at<uchar>(MidHip2D[finalIndex]);
                double rightHipValue = depth_cv.at<uchar>(RHip2D[finalIndex]);
                double leftHipValue = depth_cv.at<uchar>(LHip2D[finalIndex]);

                valueVector.insert(std::make_pair(neckValue, neck2D[finalIndex]));
                valueVector.insert(std::make_pair(leftWristValue, leftWrist2D[finalIndex]));
                valueVector.insert(std::make_pair(rightWristValue, rightWrist2D[finalIndex]));
                valueVector.insert(std::make_pair(leftElbowValue, leftElbow2D[finalIndex]));
                valueVector.insert(std::make_pair(rightElbowValue, rightElbow2D[finalIndex]));
                //valueVector.insert(std::make_pair(midHipValue, MidHip2D[finalIndex]));
                valueVector.insert(std::make_pair(rightHipValue, RHip2D[finalIndex]));
                valueVector.insert(std::make_pair(leftHipValue, LHip2D[finalIndex]));

                double minLocVal, maxLocVal;
                cv::Point minLoc, maxLoc;
                cv::minMaxLoc( depth_cv, &minLocVal, &maxLocVal, &minLoc, &maxLoc );

                //yDebug() << midHipValue << rightHipValue << leftHipValue << neckValue << leftWristValue << rightWristValue << leftElbowValue << rightElbowValue;
                yDebug() << "Values " << neckValue << noseValue << leftWristValue << rightWristValue << leftElbowValue << rightElbowValue << rightHipValue << leftHipValue;

                std::map<double, cv::Point>::iterator it = valueVector.begin();

                double max = 0.0;
                cv::Point maxPoint;
                while(it != valueVector.end())
                {
                    if (it->first>max)
                    {
                        max = it->first;
                        maxPoint = it->second;
                    }
                    it++;
                }

                yDebug() << "Max value: " << max << " max Point " << maxPoint.x << maxPoint.y;

                double maxValThreshed = (max - 3);

                cv::Mat cleanedImg;
                cv::threshold(depth_cv, cleanedImg, maxValThreshed, 255, cv::THRESH_BINARY);

                cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
                dilate(cleanedImg, cleanedImg, kernel1);

                std::vector<std::vector<cv::Point> > cnt;

                findContours(cleanedImg, cnt, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

                std::vector<cv::Moments> mu( cnt.size() );
                std::vector<cv::Point2f> mc( cnt.size() );

                std::vector<std::vector<cv::Point> > contours_poly( cnt.size() );
                std::vector<cv::Rect> boundRect( cnt.size() );

                yarp::os::Bottle &blobs  = blobPort.prepare();
                blobs.clear();

                double minValue = 1000;
                int minValueIndex = -1;

                for( size_t i = 0; i< cnt.size(); i++ )
                {
                    mu[i] = moments( cnt[i], false );
                    mc[i] = cv::Point2f( (float)(mu[i].m10/mu[i].m00), (float)(mu[i].m01/mu[i].m00) );
                    double res = cv::norm(cv::Point((int)(mc[i].x), (int)(mc[i].y)) - maxPoint);

                    yDebug() << "res" << res;
                    if (res<minValue)
                    {
                        minValue = res;
                        minValueIndex = i;
                    }
                }
                yDebug() << "res minValue " << minValue << "index" << minValueIndex;

                double checkValueOne = 0.0;
                double checkValueTwo = 0.0;

                if (neckValue > 0 && noseValue > 0)
                {
                    yError() << "USING NECK";
                    checkValueOne = fabs(max-neckValue);
                    checkValueTwo = fabs(max-noseValue);
                }
                else
                {
                    yError() << "USING HIP ";
                    checkValueOne = fabs(max-rightHipValue);
                    checkValueTwo = fabs(max-leftHipValue);
                }

                yError() << mc[minValueIndex].x << mc[minValueIndex].y;
                yError() << maxPoint.x << maxPoint.y;
                yError() << "Distance from " << minValueIndex << " is " << minValue;
                yError() << "checkValueOne vs max " << checkValueOne;
                yError() << "checkvalueTwo vs max " << checkValueTwo;

                if ( checkValueOne >= 15 || checkValueTwo >= 15 )
                {
                    approxPolyDP( cv::Mat(cnt[minValueIndex]), contours_poly[minValueIndex], 3, true );
                    boundRect[minValueIndex] = boundingRect( cv::Mat(contours_poly[minValueIndex]) );

                    drawContours(depth_cv, cnt, minValueIndex, cv::Scalar(255), 8);
                    drawContours(overlayObject, cnt, minValueIndex, cv::Scalar(216, 102 , 44), -1);
                    drawContours(segment_cv, cnt, minValueIndex, cv::Scalar(255), -1);
                    rectangle( out_cv, boundRect[minValueIndex].tl(), boundRect[minValueIndex].br(), cv::Scalar(216, 102 , 44), 2, 8, 0 );

                    yarp::os::Bottle &tmp = blobs.addList();
                    tmp.addInt32( boundRect[minValueIndex].tl().x );
                    tmp.addInt32( boundRect[minValueIndex].tl().y );
                    tmp.addInt32( boundRect[minValueIndex].br().x );
                    tmp.addInt32( boundRect[minValueIndex].br().y );

                    blobPort.setEnvelope(stamp);
                    blobPort.write();

                    double opacity = 0.4;
                    cv::addWeighted(overlayObject, opacity, out_cv, 1 - opacity, 0, out_cv);
                }
                //send arm trigger
                if (index > -1 && isArmLifted==false)
                {
                    if (armPort.getOutputCount() > 0)
                    {
                        yarp::os::Bottle cmd,rep;
                        cmd.addString("start");
                        isArmLifted = true;
                        armPort.write(cmd,rep);
                    }
                }
                else if (isArmLifted==true && index < 0)
                {
                    if (armPort.getOutputCount() > 0)
                    {
                        yarp::os::Bottle cmd,rep;
                        cmd.addString("stop");
                        isArmLifted = false;
                        armPort.write(cmd,rep);
                    }
                }
            }
            else
                followSkeletonIndex = -1;
        }

        yDebug() << "sending images";

        outImage = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(out_cv);
        outDepth = yarp::cv::fromCvMat<yarp::sig::PixelMono>(depth_cv);
        outSegment = yarp::cv::fromCvMat<yarp::sig::PixelMono>(segment_cv);

        imageOutPort.setEnvelope(stamp);
        imageOutPort.write();
        imageOutDepthPort.setEnvelope(stamp);
        imageOutDepthPort.write();
        imageOutSegmentPort.setEnvelope(stamp);
        imageOutSegmentPort.write();

        yDebug() << "done loop";
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public humanStructure_IDLServer
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

    double minVal;
    double maxVal;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    Module() : closing(false) { }

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("humanStructure"), "module name (string)").asString();
        minVal = rf.check("minVal", yarp::os::Value(0.5), "minimum value for depth (double)").asFloat64();
        maxVal = rf.check("maxVal", yarp::os::Value(3.5), "maximum value for depth (double)").asFloat64();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        processing = new Processing( moduleName );
        /* now start the thread to do the work */
        processing->open();

        processing->setDepthValues(minVal, maxVal);

        attach(rpcPort);

        return true;
    }

    /********************************************************/
    int getArmThresh()
    {
        return processing->armthresh;
    }

    /********************************************************/
    bool setArmThresh(const int32_t threshold)
    {
        bool val = true;

        if (threshold > 0)
            processing->armthresh = threshold;
        else
            val = false;

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool quit(){
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setDefaultContext("humanStructure");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    return module.runModule(rf);
}
