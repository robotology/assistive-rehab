/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file helpers.h
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#ifndef ASSISTIVE_REHAB_HELPERS_H
#define ASSISTIVE_REHAB_HELPERS_H

#include <yarp/sig/Image.h>

namespace assistive_rehab
{
    void filterDepth(const yarp::sig::ImageOf<yarp::sig::PixelFloat> &src,
                     yarp::sig::ImageOf<yarp::sig::PixelFloat> &dst,
                     const int kernelSize, const int iterations,
                     const float min_dist, const float max_dist);
}

#endif
