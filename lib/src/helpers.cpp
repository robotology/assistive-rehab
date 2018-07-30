/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file helpers.cpp
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <limits>
#include <opencv2/opencv.hpp>
#include "AssistiveRehab/helpers.h"

using namespace std;
using namespace yarp::sig;
using namespace assistive_rehab;

void assistive_rehab::filterDepth(const ImageOf<PixelFloat> &src, ImageOf<PixelFloat> &dst,
                                  const int kernelSize, const int iterations,
                                  const float min_dist, const float max_dist)
{
    if ((dst.width()!=src.width()) || (dst.height()!=src.height()))
    {
        dst.resize(src);
    }

    for (size_t y=0; y<src.height(); y++)
    {
        for (size_t x=0; x<src.width(); x++)
        {
            dst.pixel(x,y)=(src.pixel(x,y)>=min_dist ? src.pixel(x,y) : numeric_limits<float>::max());
        }
    }

    cv::Mat dstMat=cv::cvarrToMat(dst.getIplImage());
    cv::Mat kernel=cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(kernelSize,kernelSize));
    cv::erode(dstMat,dstMat,kernel,cv::Point(-1,-1),iterations);
    cv::threshold(dstMat,dstMat,max_dist,0,cv::THRESH_TOZERO_INV);
}

