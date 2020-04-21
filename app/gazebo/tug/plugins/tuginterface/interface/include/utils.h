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

struct Velocity
{
    double lin_vel;
    double ang_vel;
    Velocity() : lin_vel(0.0), ang_vel(0.0) {}
    Velocity(const double _lin_vel, const double _ang_vel) : lin_vel(_lin_vel), ang_vel(_ang_vel) {}
};

void updateScript(sdf::ElementPtr &actor_sdf, const std::map<double, ignition::math::Pose3d> &m);

std::map<double, ignition::math::Pose3d> createMap(const yarp::sig::Matrix &t, const Velocity &vel);

std::map<double, ignition::math::Pose3d> generateWaypoints(const Velocity &vel,
                                                           const std::map<double, ignition::math::Pose3d> &m);

#endif
