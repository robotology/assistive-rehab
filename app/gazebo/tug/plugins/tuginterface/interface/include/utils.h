// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Vasco
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

#ifndef GAZEBO_ASSISTIVEREHAB_UTILS
#define GAZEBO_ASSISTIVEREHAB_UTILS

#include <yarp/sig/Vector.h>

#include <gazebo/gazebo.hh>

void updateScript(sdf::ElementPtr &actor_sdf, const std::map<double, ignition::math::Pose3d> &m);

std::map<double, ignition::math::Pose3d> createMap(const yarp::sig::Matrix &t,const double &vel);

std::map<double, ignition::math::Pose3d> generateWaypoints(const int ntot, const double &vel,
                                                           const std::map<double, ignition::math::Pose3d> &m);

#endif
