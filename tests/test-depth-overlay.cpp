/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-depth-overlay.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Image.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class Overlayer : public RFModule
{
    BufferedPort<ImageOf<PixelFloat>> depthPort;
    BufferedPort<ImageOf<PixelRgb>>   rgbPort;
    BufferedPort<Bottle>              keysPort;
    BufferedPort<ImageOf<PixelRgb>>   ovlPort;

    ImageOf<PixelRgb> depth;
    ImageOf<PixelRgb> rgb;
    Bottle            keys;

    double min_d,max_d;
    double alpha,beta;

    void greenify(const ImageOf<PixelFloat> &dist)
    {
        depth.resize(dist);
        for (size_t y=0; y<depth.height(); y++)
        {
            for (size_t x=0; x<depth.width(); x++)
            {
                unsigned char green=0;
                float d=dist.pixel(x,y);
                if ((d>=min_d) && (d<=max_d))
                {
                    green=(unsigned char)(255.0*(1.0-(d-min_d)/(max_d-min_d)));
                }
                depth.pixel(x,y)=PixelRgb(0,green,0);
            }
        }
    }

    void addKeys(const cv::Mat &img)
    {
        if (Bottle *b1=keys.get(0).asList())
        {
            for (size_t i=0; i<b1->size(); i++)
            {
                if (Bottle *b2=b1->get(i).asList())
                {
                    for (size_t i=0; i<b2->size(); i++)
                    {
                        if (Bottle *k=b2->get(i).asList())
                        {
                            if (k->size()==4)
                            {
                                int u=(int)k->get(1).asDouble();
                                int v=(int)k->get(2).asDouble();
                                cv::circle(img,cv::Point(u,v),2,cv::Scalar(255,0,0));
                            }
                        }
                    }
                }
            }
        }
    }

    bool configure(ResourceFinder &rf) override
    {
        min_d=rf.check("min-d",Value(1.5)).asDouble();
        max_d=rf.check("max-d",Value(3.0)).asDouble();

        alpha=rf.check("alpha",Value(0.5)).asDouble();
        beta=rf.check("beta",Value(0.5)).asDouble();

        depthPort.open("/test-depth-overlay/depth:i");
        rgbPort.open("/test-depth-overlay/rgb:i");
        keysPort.open("/test-depth-overlay/keys:i");
        ovlPort.open("/test-depth-overlay/rgb:o");

        return true;
    }

    double getPeriod() override
    {
        return (1.0/30.0);
    }

    bool updateModule() override
    {
        if (ImageOf<PixelFloat> *depth=depthPort.read(false))
        {
            greenify(*depth);
        }

        if (ImageOf<PixelRgb> *rgb=rgbPort.read(false))
        {
            this->rgb=*rgb;
        }

        if (Bottle *keys=keysPort.read(false))
        {
            this->keys=*keys;
        }

        if ((depth.width()==rgb.width()) && (depth.width()>0) &&
            (depth.height()==rgb.height()) && (depth.height()>0) &&
            (keys.size()>0))
        {
            ImageOf<PixelRgb> &ovl=ovlPort.prepare();
            ovl.resize(rgb);

            cv::Mat src1=cv::cvarrToMat(rgb.getIplImage());
            cv::Mat src2=cv::cvarrToMat(depth.getIplImage());
            cv::Mat dst=cv::cvarrToMat(ovl.getIplImage());
            cv::addWeighted(src1,alpha,src2,beta,0.0,dst);
            addKeys(dst);

            ovlPort.writeStrict();
        }
        return true;
    }

    bool close() override
    {
        depthPort.close();
        rgbPort.close();
        keysPort.close();
        ovlPort.close();
        return true;
    }
};

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

    Overlayer overlayer;
    return overlayer.runModule(rf);
}

