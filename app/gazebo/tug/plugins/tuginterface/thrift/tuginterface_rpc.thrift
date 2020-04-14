# Copyright (C) 2015 iCub Facility
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

service TugInterfaceServer
{

    /** 
    * Start script for moving actor.
    * @return returns true or false on success / failure
    */
    bool start();

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
     * Pause actor for time seconds.
     * @param time seconds during which actor is paused.
     * @return returns walking speed
     */
    bool pause(1: double time);

}
