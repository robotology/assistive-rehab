/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * \defgroup helpers helpers
 *
 * Helper functions.
 *
 * \section intro_sec Description
 *
 * The current implemented helper function is required to filter a depth image.
 *
 * \author Ugo Pattacini <ugo.pattacini@iit.it>
 */

#ifndef ASSISTIVE_REHAB_HELPERS_H
#define ASSISTIVE_REHAB_HELPERS_H

#include <yarp/sig/Image.h>

namespace assistive_rehab
{
    /**
    * Erode a depth image. After erosion, only pixels within min_dist and max_dist are kept.
    * @param src input depth image.
    * @param dst output depth image.
    * @param kernelSize size of the applied kernel.
    * @param iterations number of times erosion is applied.
    * @param min_dist threshold on the minimum distance.
    * @param max_dist threshold on the maximum distance.
    */
    void filterDepth(const yarp::sig::ImageOf<yarp::sig::PixelFloat> &src,
                     yarp::sig::ImageOf<yarp::sig::PixelFloat> &dst,
                     const int kernelSize, const int iterations,
                     const float min_dist, const float max_dist);
}

#endif
