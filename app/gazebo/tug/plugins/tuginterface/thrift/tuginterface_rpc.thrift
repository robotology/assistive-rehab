# Copyright (C) 2015 iCub Facility
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

struct Pose 
{
1: double x;     /* x position [m] */
2: double y;     /* y position [m] */
3: double theta;   /* rotation along yaw axis [deg]*/
}

struct Animation 
{
1: string name="";     /* animation name */
2: i32 id=-1;          /* animation id */
}

service TugInterfaceServer
{

    /**
    * Stop script for moving actor.
    * @return returns true or false on success / failure
    */
    bool stop();

    /**
     * Set walking speed for reaching the target.
     * @param speed velocity to set.
     * @return returns true or false on success / failure
     */
    bool setSpeed(1: double speed);

    /**
     * Get walking speed.
     * @return returns walking speed
     */
    double getSpeed();

    /**
     * Get the list of animations associated with the actor.
     * @return returns list of animations associated with the actor.
     */
    list<string>getAnimationList();

    /**
     * Play specified animation.
     * @param animation in the form (name, id). If not specified, all animations are played according to their id. The id has to be specified if the animation is played several times during the script.
     * @param complete if true, the whole script is played starting from the specified animation.
     * @return returns true / false on success / failure.
     */
    bool play(1: Animation animation, 2: bool complete=false);

    /**
     * Pause actor for time seconds.
     * @param time seconds during which actor is paused.
     * @return returns walking speed
     */
    bool pause(1: double time);

    /**
     * Reach a target location.
     * @param p pose in the form x,y,theta.
     * @return true/false on success/failure.
     */
    bool goTo(1: Pose p);

}
