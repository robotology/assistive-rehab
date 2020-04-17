# Copyright (C) 2015 iCub Facility
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

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
     * @param name string indicating the animation. If not specified, all animations are played according to their id.
     * @param complete if true, the whole script is played starting from the specified animation.
     * @param id int indicating the id of animation. To be specified if the animation is played several times during the script.
     * @return returns true / false on success / failure.
     */
    bool play(1: string name="", 2: bool complete=false, 3: i32 id=-1);

    /**
     * Pause actor for time seconds.
     * @param time seconds during which actor is paused.
     * @return returns walking speed
     */
    bool pause(1: double time);

}
