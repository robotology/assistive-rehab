/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file idl.thrift
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

struct Property { }
(
   yarp.name="yarp::os::Property"
   yarp.includefile="yarp/os/Property.h"
)

/**
 * navController_IDL
 *
 * IDL Interface to Navigation Controller services.
 */
service navController_IDL
{
   /**
    * Reach for a target location.
    * @param x is the x-coordinate of the target location (meters).
    * @param y is the y-coordinate of the target location (meters).
    * @param theta is the theta-coordinate of the target location (degrees).
    * @param heading_rear is true to specify if the robot has to drive backward.
    * @return true/false on success/failure.
    */
   bool go_to(1:double x, 2:double y, 3:double theta, 4:bool heading_rear = false);

   /**
    * Blocking version of reach for a target location. The service returns ack only
    * when navigation is done.
    * @param x is the x-coordinate of the target location (meters).
    * @param y is the y-coordinate of the target location (meters).
    * @param theta is the theta-coordinate of the target location (degrees).
    * @param heading_rear is true to specify if the robot has to drive backward.
    * @param timeout sets up optionally a timeout given in seconds, if > 0.
    * @return true/false on success/failure.
    */
   bool go_to_wait(1:double x, 2:double y, 3:double theta, 4:bool heading_rear = false,
                   5:i32 timeout = 0);

   /**
    * Start navigation while controlling distance from the specified skeleton.
    * @param skeleton_tag is the skeleton's tag.
    * @return true/false on success/failure.
    */
   bool track_skeleton(1:string skeleton_tag);

   /**
    * Query if navigation is underway.
    * @return true if navigation is being currently performed.
    */
   bool is_navigating();

   /**
    * Stop navigation.
    * @return true/false on success/failure.
    */
   bool stop();

   /**
    * Reset odometry.
    * @return true/false on success/failure.
    */
   bool reset_odometry();

   /**
    * Query which skeleton is currently under control.
    * @return the name of the skeleton; empty otherwise.
    */
   string which_skeleton();

   /**
    * Retrieve internal state.
    * @return a property-like object containing the state
    *         in the form (robot-state {idle|track|nav}) (robot-location (x y theta))
    *         (robot-velocity (v_x v_theta)) [(target-location (x y theta heading))]
    *         [(skeleton-tag tag) (skeleton-location (x y))].
    */
   Property get_state();
}
