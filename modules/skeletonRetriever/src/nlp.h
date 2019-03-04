/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file nlp.h
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#ifndef NLP_H
#define NLP_H

#include <vector>
#include <utility>
#include <yarp/sig/Vector.h>
#include "AssistiveRehab/skeleton.h"
#include "utils.h"


/****************************************************************/
struct LimbOptimizer
{
    /****************************************************************/
    static std::vector<std::pair<std::string,std::pair<yarp::sig::Vector,yarp::sig::Vector>>> optimize(const CamParamsHelper &camParams,
                                                                                                       const assistive_rehab::KeyPoint* k,
                                                                                                       const std::vector<double>& lengths);
};

#endif

