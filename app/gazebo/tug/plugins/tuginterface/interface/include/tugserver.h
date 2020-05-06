// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2015 iCub Facility
 * Authors: Valentina Vasco
 * CopyPolicy: Released under the terms of the LGPLv2.1 or any later version, see LGPL.TXT or LGPL3.TXT
 *
 */

#ifndef GAZEBO_ASSISTIVEREHAB_TUGSERVER
#define GAZEBO_ASSISTIVEREHAB_TUGSERVER

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Actor.hh>

#include <include/utils.h>

#include <TugInterfaceServer.h>
#include <Pose.h>
#include <Animation.h>

class TugServer: public TugInterfaceServer
{
private:
    gazebo::physics::WorldPtr world;
    gazebo::physics::ActorPtr actor;
    Velocity vel;
    yarp::sig::Matrix targets;
    yarp::sig::Matrix line_frame;

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
     * Get model position as defined in world, with respect to start-line.
     * @param model_name name string defining the name of the model.
     * @return a property-like object in the form
     *         (pose_world (x y z ax ay az theta)).
     */
    virtual yarp::os::Property getModelPos(const std::string &model_name);

    /**
     * Pause actor.
     * @param time [optional] seconds during which actor is paused (if time > 0).
     * @return returns true / false on success / failure.
     */
    virtual bool pause(const double time);

    /**
     * Play from last animation.
     * @return returns true / false on success / failure.
     */
    virtual bool playFromLast();

    /**
     * Get current animation being played.
     * @return returns string defining the current animation being played.
     */
    virtual std::string getState();

    /**
     * Reach a target location.
     * @param p pose in the form x,y,theta.
     * @return true/false on success/failure.
     */
    virtual bool goTo(const Pose &p);

    /**
     * Blocking version of reach for a target location. The service returns ack only
     * when target is reached.
     * @param p pose in the form x,y,theta.
     * @return true/false on success/failure.
     */
    virtual bool goToWait(const Pose &p);

    /**
     * Reach a sequence of targets.
     * @param p list of poses in the form x1,y1,theta1,x2,y2,theta2.
     * @return true/false on success/failure.
     */
    virtual bool goToSeq(const std::vector<double> &p);

    /**
     * Set target to reach during walk animation.
     * @param p pose in the form x,y,theta.
     * @return returns true or false on success / failure
     */
    virtual bool setTarget(const Pose &p);

    void updateMap(const yarp::sig::Matrix &t);

    void init(const Velocity &vel, const yarp::sig::Matrix &waypoints);

    void attachWorldPointer(gazebo::physics::WorldPtr p);

    void attachActorPointer(gazebo::physics::ActorPtr p);

};

#endif
