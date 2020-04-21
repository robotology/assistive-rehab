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

#include <include/utils.h>
#include <../include/TugInterfaceServer.h>
#include <../include/Pose.h>
#include <../include/Animation.h>

class TugServer: public TugInterfaceServer
{
private:
    gazebo::physics::WorldPtr world;
    gazebo::physics::ActorPtr actor;
    Velocity vel;
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
     * @param animation in the form (name, id). If not specified, all animations are played according to their id.
     * The id has to be specified if the animation is played several times during the script.
     * @param complete if true, the whole script is played starting from the specified animation.
     * @return returns true / false on success / failure.
     */
    virtual bool play(const Animation &animation, const bool complete);

    /**
     * Pause actor for time seconds.
     * @param time seconds during which actor is paused.
     * @return returns walking speed
     */
    virtual bool pause(const double time);

    /**
     * Reach a target location.
     * @param p pose in the form x,y,theta.
     * @return true/false on success/failure.
     */
    virtual bool goTo(const Pose &p);

    void updateMap(const yarp::sig::Matrix &t);

    void init(const Velocity &vel, const yarp::sig::Matrix &waypoints);

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
