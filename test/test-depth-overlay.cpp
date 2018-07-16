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
#include <yarp/sig/Image.h>

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;

class Overlayer : public RFModule
{
    BufferedPort<ImageOf<PixelFloat>> depthPort;
    BufferedPort<ImageOf<PixelRgb>> rgbPort;
    BufferedPort<ImageOf<PixelRgb>> ovlPort;

    ImageOf<PixelRgb> depth;
    ImageOf<PixelRgb> rgb;

    double min_d;
    double max_d;

    void process(const ImageOf<PixelFloat> &dist)
    {
        depth.resize(dist);
        for (size_t y=0; y<depth.height(); y++)
        {
            for (size_t x=0; x<depth.width(); x++)
            {
                unsigned char l=0;
                float d=dist.pixel(x,y);
                if ((d>=min_d) && (d<=max_d))
                {
                    l=(unsigned char)(255*(1.0-(d-min_d)/(max_d-min_d)));
                }
                depth.pixel(x,y)=PixelRgb(l,l,0);
            }
        }
    }

    bool configure(ResourceFinder &rf) override
    {
        min_d=rf.check("min-d",Value(1.5)).asDouble();
        max_d=rf.check("max-d",Value(3.0)).asDouble();

        depthPort.open("/test-depth-overlay/depth:i");
        rgbPort.open("/test-depth-overlay/rgb:i");
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
            process(*depth);
        }

        if (ImageOf<PixelRgb> *rgb=rgbPort.read(false))
        {
            this->rgb=*rgb;
        }

        if ((depth.width()==rgb.width()) && (depth.width()>0) &&
            (depth.height()==rgb.height()) && (depth.height()>0))
        {
            ImageOf<PixelRgb> &ovl=ovlPort.prepare();
            ovl.resize(rgb);

            Mat src1=cvarrToMat(rgb.getIplImage());
            Mat src2=cvarrToMat(depth.getIplImage());
            Mat dst=cvarrToMat(ovl.getIplImage());

            double alpha=0.5;
            addWeighted(src1,alpha,src2,1.0-alpha,0.0,dst);

            ovlPort.writeStrict();
        }
        return true;
    }

    bool close() override
    {
        depthPort.close();
        rgbPort.close();
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

