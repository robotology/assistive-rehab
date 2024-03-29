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

struct Property { }
(
   yarp.name="yarp::os::Property"
   yarp.includefile="yarp/os/Property.h"
)

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
     * Get time taken to complete the specified animation.
     * @param animation_name name of the animation defined in the sdf.
     * @param nsamples total number of samples for computing the average time.
     * @return returns animation time.
     */
    double getTime(1:string animation_name, 2:i32 nsamples);

    /**
     * Get walking time.
     * @return returns walking time
     */
    double getWalkingTime();

    /**
     * Get stand up plus sit down time.
     * @return returns stand up plus sit down time
     */
    double getStandSitTime();
    
    /**
     * Get number of steps.
     * @return returns number of steps
     */
    i32 getNumSteps();

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
     * Get model position as defined in world, with respect to start-line.
     * @param model_name name string defining the name of the model.
     * @return a property-like object in the form
     *         (pose_world (x y z ax ay az theta)).
     */
    Property getModelPos(1: string model_name);

    /**
     * Pause actor.
     * @param time [optional] seconds during which actor is paused (if time > 0).
     * @return returns true / false on success / failure.
     */
    bool pause(1: double time=0.0);

    /**
     * Returns true is actor is active.
     * @return returns true / false if actor is active / paused.
     */
    bool isActive();

    /**
     * Play from last animation.
     * @param complete if true, the whole script is played starting from last stop.
     * @return returns true / false on success / failure.
     */
    bool playFromLast(1: bool complete=false);

    /**
     * Get the torso front surface x coordinate.
     * @return returns double defining the torso front surface x coordinate.
     */
    double getTorsoFront();

    /**
     * Get current animation being played.
     * @return returns string defining the current animation being played.
     */
    string getState();

    /**
     * Reach a target location.
     * @param p pose in the form x,y,theta.
     * @return true/false on success/failure.
     */
    bool goTo(1: Pose p);

    /**
     * Blocking version of reach for a target location. The service returns ack only
     * when target is reached.
     * @param p pose in the form x,y,theta.
     * @return true/false on success/failure.
     */
    bool goToWait(1: Pose p);

    /**
     * Reach a sequence of targets.
     * @param p list of poses in the form x1,y1,theta1,x2,y2,theta2.
     * @return true/false on success/failure.
     */
    bool goToSeq(1: list<double> p);

    /**
     * Set target to reach during walk animation.
     * @param p pose in the form x,y,theta.
     * @return returns true or false on success / failure
     */
    bool setTarget(1: Pose p);
}
