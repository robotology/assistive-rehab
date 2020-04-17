// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Vasco
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

#ifndef GAZEBO_ASSISTIVEREHAB_TUGSERVER
#define GAZEBO_ASSISTIVEREHAB_TUGSERVER

#include <yarp/sig/Matrix.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Actor.hh>

#include <../include/TugInterfaceServer.h>

class TugServer: public TugInterfaceServer
{
private:
    gazebo::physics::WorldPtr world;
    gazebo::physics::ActorPtr actor;
    double lin_speed;
    double ang_speed;
    yarp::sig::Matrix targets;

public:
    TugServer();
    ~TugServer();

    /**
     * Stop script for moving actor.
     * @return returns true or false on success / failure
     */
    virtual bool stop();

    /**
     * Set walking speed for reaching the target.
     * @param speed velocity to set.
     * @return returns true or false on success / failure
     */
    virtual bool setSpeed(const double speed);

    /**
     * Get walking speed.
     * @return returns walking speed.
     */
    virtual double getSpeed();

    /**
     * Get the list of animations associated with the actor.
     * @return returns list of animations associated with the actor.
     */
    virtual std::vector<std::string> getAnimationList();

    /**
     * Play specified animation.
     * @param name string indicating the animation. If not specified, all animations are played according to their id.
     * @param complete if true, the whole script is played starting from the specified animation.
     * @param id int indicating the id of animation. To be specified if the animation is played several times during the script.
     * @return returns true / false on success / failure.
     */
    virtual bool play(const std::string &name, const bool complete, const int id);

    /**
     * Pause actor for time seconds.
     * @param time seconds during which actor is paused.
     * @return returns walking speed
     */
    virtual bool pause(const double time);

    /**
     * Reach a target location.
     * @param x is the x-coordinate of the target location (meters).
     * @param y is the y-coordinate of the target location (meters).
     * @param theta is the theta-coordinate of the target location (degrees).
     * @return true/false on success/failure.
     */
    virtual bool goTo(const double x, const double y, const double theta);

    void updateMap(const yarp::sig::Matrix &t);

    void init(const double &lin_speed, const double &ang_speed,
              const yarp::sig::Matrix &waypoints);

    void attachWorldPointer(gazebo::physics::WorldPtr p)
    {
        world=p;
    }

    void attachActorPointer(gazebo::physics::ActorPtr p)
    {
        actor=p;
    }

};

#endif
