/*
 * Copyright (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
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

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <cstring>
#include <vector>
#include <iostream>
#include <utility>

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::os::Bottle>
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imageOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle >    targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle >    blobPort;

public:
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
    bool open(){

        this->useCallback();

        BufferedPort<yarp::os::Bottle >::open( "/" + moduleName + "/skeleton:i" );
        imageInPort.open("/" + moduleName + "/image:i");

        imageOutPort.open("/" + moduleName + "/image:o");
        targetPort.open("/" + moduleName + "/target:o");
        blobPort.open("/" + moduleName + "/blobs:o");

        return true;
    }

    /********************************************************/
    void close()
    {
        imageOutPort.close();
        imageInPort.close();
        BufferedPort<yarp::os::Bottle >::close();
        targetPort.close();
        blobPort.close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle >::interrupt();
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &data )
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = imageOutPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = imageInPort.read();

        yarp::os::Bottle &target  = targetPort.prepare();

        int skeletonSize = data.get(0).asList()->size();
        int internalElements = 0;

        if (skeletonSize>0)
        {
            target = data;
            internalElements = data.get(0).asList()->get(0).asList()->size();
        }

        outImage = *inImage;

        outImage.resize(inImage->width(), inImage->height());

        std::vector<cv::Point> neck2D;
        std::vector<cv::Point> nose2D;
        std::vector<cv::Point> rightEar2D;
        std::vector<cv::Point> leftEar2D;
        std::vector<cv::Point> leftShoulder2D;
        std::vector<cv::Point> rightShoulder2D;
        std::vector<cv::Point> rightWrist2D;
        std::vector<cv::Point> leftWrist2D;
        cv::Point point;

        std::vector<yarp::os::Bottle> shapes;
        std::vector<std::pair <int,int> > elements;

        for (int i = 0; i < skeletonSize; i++)
        {
            if (yarp::os::Bottle *propField = data.get(0).asList()->get(i).asList())
            {
                for (int ii = 0; ii < internalElements; ii++)
                {
                    if (yarp::os::Bottle *propFieldPos = propField->get(ii).asList())
                    {
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"REar") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asDouble();
                            point.y = (int)propFieldPos->get(2).asDouble();
                            rightEar2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LEar") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asDouble();
                            point.y = (int)propFieldPos->get(2).asDouble();
                            leftEar2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"Neck") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asDouble();
                            point.y = (int)propFieldPos->get(2).asDouble();
                            neck2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"Nose") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asDouble();
                            point.y = (int)propFieldPos->get(2).asDouble();
                            nose2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LShoulder") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asDouble();
                            point.y = (int)propFieldPos->get(2).asDouble();
                            leftShoulder2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RShoulder") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asDouble();
                            point.y = (int)propFieldPos->get(2).asDouble();
                            rightShoulder2D.push_back(point);
                        }

                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"RWrist") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asDouble();
                            point.y = (int)propFieldPos->get(2).asDouble();
                            rightWrist2D.push_back(point);
                        }
                        if ( std::strcmp (propFieldPos->get(0).asString().c_str(),"LWrist") == 0)
                        {
                            point.x = (int)propFieldPos->get(1).asDouble();
                            point.y = (int)propFieldPos->get(2).asDouble();
                            leftWrist2D.push_back(point);
                        }
                    }
                }
            }
        }

        cv::Mat out_cv = cv::cvarrToMat((IplImage *)outImage.getIplImage());

        //need this increment as case might be that skeleton does not
        //satisfy conditions to fill in bottle
        int increment = 0;

        for (size_t i = 0; i < skeletonSize; i++)
        {
            cv::Point topLeft;
            cv::Point bottomRight;

            int length = 0;
            int shift = 10;

            if (nose2D[i].x > 0)
            {
                if (leftEar2D[i].x > 0 && rightEar2D[i].x > 0)
                {
                    length = leftEar2D[i].x - rightEar2D[i].x;
                    topLeft.x = rightEar2D[i].x - shift;
                    topLeft.y = rightEar2D[i].y - length;

                    bottomRight.x = leftEar2D[i].x + shift;
                    bottomRight.y = leftEar2D[i].y + length;
                }
                else if (leftEar2D[i].x == 0 && rightEar2D[i].x > 0)
                {
                    length = nose2D[i].x - rightEar2D[i].x;
                    topLeft.x = rightEar2D[i].x - shift;
                    topLeft.y = rightEar2D[i].y - length;

                    bottomRight.x = nose2D[i].x + shift;
                    bottomRight.y = nose2D[i].y + length;
                }
                else if (rightEar2D[i].x == 0 && leftEar2D[i].x > 0)
                {
                    length = leftEar2D[i].x - nose2D[i].x;
                    topLeft.x = nose2D[i].x - shift;
                    topLeft.y = nose2D[i].y - length;

                    bottomRight.x = leftEar2D[i].x + shift;
                    bottomRight.y = leftEar2D[i].y + length;
                }

                if (topLeft.x < 1)
                    topLeft.x = 1;
                else if (topLeft.x > 319)
                    topLeft.x = 319;
                else if (topLeft.y < 1)
                    topLeft.y = 1;
                else if (topLeft.y > 239)
                    topLeft.y = 239;

                if (bottomRight.x < 1)
                    bottomRight.x = 1;
                else if (bottomRight.x > 319)
                    bottomRight.x = 319;
                else if (bottomRight.y < 1)
                    bottomRight.y = 1;
                else if (bottomRight.y > 239)
                    bottomRight.y = 239;

                circle(out_cv, topLeft, 3, cv::Scalar(0, 255 , 0), 1, 8);
                circle(out_cv, bottomRight, 3, cv::Scalar(0, 255 , 0), 1, 8);

                cv::rectangle(out_cv, topLeft, bottomRight, cv::Scalar(0, 255, 0), 2, 8);

                yarp::os::Bottle tmp;
                yInfo() << "###########";
                yInfo() << topLeft.x << topLeft.y << bottomRight.x << bottomRight.y;

                if (topLeft.x < bottomRight.x && topLeft.y < bottomRight.y)
                {
                    tmp.addInt(topLeft.x);
                    tmp.addInt(topLeft.y);
                    tmp.addInt(bottomRight.x);
                    tmp.addInt(bottomRight.y);
                    yInfo() << "IN NORMAL" << tmp.toString();
                    elements.push_back(std::make_pair(topLeft.x,increment));
                    shapes.push_back(tmp);
                    increment++;
                }
                else
                {
                    tmp.addInt(topLeft.x);
                    tmp.addInt(topLeft.y);
                    tmp.addInt(bottomRight.x);
                    tmp.addInt(bottomRight.y);
                    yError() << "WTF NORMAL" << tmp.toString();
                }
            }
            else
            {
                if (leftEar2D[i].x > 0 && rightEar2D[i].x > 0)
                {
                    length = rightEar2D[i].x - leftEar2D[i].x;
                    topLeft.x = leftEar2D[i].x - shift;
                    topLeft.y = leftEar2D[i].y - length;

                    bottomRight.x = rightEar2D[i].x + shift;
                    bottomRight.y = rightEar2D[i].y + length;

                    if (topLeft.x < 1)
                        topLeft.x = 1;
                    else if (topLeft.x > 319)
                        topLeft.x = 319;
                    else if (topLeft.y < 1)
                        topLeft.y = 1;
                    else if (topLeft.y > 239)
                        topLeft.y = 239;

                    if (bottomRight.x < 1)
                        bottomRight.x = 1;
                    else if (bottomRight.x > 319)
                        bottomRight.x = 319;
                    else if (bottomRight.y < 1)
                        bottomRight.y = 1;
                    else if (bottomRight.y > 239)
                        bottomRight.y = 239;

                    circle(out_cv, topLeft, 3, cv::Scalar(255, 0 , 0), 1, 8);

                    cv::rectangle(out_cv, topLeft, bottomRight, cv::Scalar( 255, 0, 0), 2, 8);

                    yarp::os::Bottle tmp;

                    if (topLeft.x < bottomRight.x && topLeft.y < bottomRight.y)
                    {
                        tmp.addInt(topLeft.x);
                        tmp.addInt(topLeft.y);
                        tmp.addInt(bottomRight.x);
                        tmp.addInt(bottomRight.y);
                        yInfo() << "IN REVERSED" << tmp.toString();
                        elements.push_back(std::make_pair(topLeft.x, increment));
                        shapes.push_back(tmp);
                        increment++;
                    }
                    else
                    {
                        tmp.addInt(topLeft.x);
                        tmp.addInt(topLeft.y);
                        tmp.addInt(bottomRight.x);
                        tmp.addInt(bottomRight.y);
                        yError() << "WTF REVERSED" << tmp.toString();
                    }
                }
            }
        }

        if (elements.size()>0)
        {
            for (int i=0; i<elements.size(); i++)
                yInfo() << "Testing elements " << i << elements[i].first << elements[i].second;

            std::sort(elements.begin(), elements.end());

            for (int i=0; i<elements.size(); i++)
                yInfo() << "Sorted elements " << i << elements[i].first << elements[i].second;

            if ( shapes.size() > 0)
            {
                yarp::os::Bottle &blobs  = blobPort.prepare();
                blobs.clear();

                yInfo() << "**************** SIZE" << elements.size() << shapes.size() ;
                for (int i=0; i<shapes.size(); i++)
                {
                    yInfo() << "**************** " << shapes[elements[i].second].toString();
                    yarp::os::Bottle &tmp = blobs.addList();
                    tmp.addInt(shapes[elements[i].second].get(0).asInt());
                    tmp.addInt(shapes[elements[i].second].get(1).asInt());
                    tmp.addInt(shapes[elements[i].second].get(2).asInt());
                    tmp.addInt(shapes[elements[i].second].get(3).asInt());
                }

                targetPort.write();
            }
        }
        imageOutPort.write();
        blobPort.write();
    }
};

/********************************************************/
class Module : public yarp::os::RFModule
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("human-structure"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;

        processing = new Processing( moduleName );
        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

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

    rf.setVerbose();
    rf.configure(argc,argv);

    return module.runModule(rf);
}
