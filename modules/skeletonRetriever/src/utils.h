/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file utils.h
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#ifndef UTILS_H
#define UTILS_H

#include <cmath>

/****************************************************************/
class CamParamsHelper
{
    size_t width,height;
    double fov_h,focal;
    CamParamsHelper()=delete;

public:
    /****************************************************************/
    CamParamsHelper(const size_t w, const size_t h, const double f) :
                    width(w), height(h), fov_h(f)
    {
        focal=width/(2.0*std::tan(fov_h*(M_PI/180.0)/2.0));
    }

    /****************************************************************/
    const size_t& get_width() const { return width; }
    const size_t& get_height() const { return height; }
    const double& get_focal() const { return focal; }
};

#endif

