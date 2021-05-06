/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file test-overlay.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <utility>
#include <opencv2/opencv.hpp>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Image.h>
#include <yarp/cv/Cv.h>
#include "AssistiveRehab/helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::cv;
using namespace assistive_rehab;

class Overlayer : public RFModule
{
    BufferedPort<ImageOf<PixelFloat>> depthPortIn;
    BufferedPort<ImageOf<PixelFloat>> depthPortOut;
    BufferedPort<ImageOf<PixelRgb>>   rgbPort;
    BufferedPort<Bottle>              keysPort;
    BufferedPort<ImageOf<PixelRgb>>   ovlPort;

    ImageOf<PixelRgb> depth;
    ImageOf<PixelRgb> rgb;
    Bottle            keys;

    int filter_depth_kernel_size;
    int filter_depth_iterations;
    float filter_depth_min_dist;
    float filter_depth_max_dist;
    double alpha,beta;

    void greenify(const ImageOf<PixelFloat> &dist)
    {
        if ((depth.width()!=dist.width()) || (depth.height()!=dist.height()))
        {
            depth.resize(dist);
        }

        float range_dist=filter_depth_max_dist-filter_depth_min_dist;
        for (size_t y=0; y<depth.height(); y++)
        {
            for (size_t x=0; x<depth.width(); x++)
            {
                unsigned char green=0;
                float d=dist.pixel(x,y);
                if (d>0.0)
                {
                    green=(unsigned char)(255.0*(1.0-(d-filter_depth_min_dist)/range_dist));
                }
                depth.pixel(x,y)=PixelRgb(0,green,0);
            }
        }
    }

    void addKeys(cv::Mat &img)
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
                                cv::circle(img,cv::Point(u,v),2,cv::Scalar(0,0,255));
                            }
                        }
                    }
                }
            }
        }
    }

    bool configure(ResourceFinder &rf) override
    {
        filter_depth_kernel_size=rf.check("filter-depth-kernel-size",Value(6)).asInt();
        filter_depth_iterations=rf.check("filter-depth-iterations",Value(4)).asInt();
        filter_depth_min_dist=(float)rf.check("filter-depth-min-dist",Value(1.0)).asDouble();
        filter_depth_max_dist=(float)rf.check("filter-depth-max-dist",Value(4.0)).asDouble();

        alpha=rf.check("alpha",Value(0.5)).asDouble();
        beta=rf.check("beta",Value(0.5)).asDouble();

        depthPortIn.open("/test-overlay/depth:i");
        depthPortOut.open("/test-overlay/depth:o");
        rgbPort.open("/test-overlay/rgb:i");
        keysPort.open("/test-overlay/keys:i");
        ovlPort.open("/test-overlay/rgb:o");

        return true;
    }

    double getPeriod() override
    {
        return (1.0/30.0);
    }

    bool updateModule() override
    {
        if (ImageOf<PixelFloat> *depthIn=depthPortIn.read(false))
        {
            ImageOf<PixelFloat> &depthOut=depthPortOut.prepare();
            filterDepth(*depthIn,depthOut,filter_depth_kernel_size,filter_depth_iterations,
                        filter_depth_min_dist,filter_depth_max_dist);
            greenify(depthOut);
            depthPortOut.writeStrict();
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

            cv::Mat src1=toCvMat(rgb);
            cv::Mat src2=toCvMat(depth);
            cv::Mat dst=toCvMat(ovl);
            cv::addWeighted(src1,alpha,src2,beta,0.0,dst);
            addKeys(dst);

            ovl=fromCvMat<PixelRgb>(dst);
            ovlPort.writeStrict();
        }
        return true;
    }

    bool close() override
    {
        depthPortIn.close();
        depthPortOut.close();
        rgbPort.close();
        keysPort.close();
        ovlPort.close();
        return true;
    }

public:
    Overlayer() { }
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

