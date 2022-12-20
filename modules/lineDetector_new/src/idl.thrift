/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file idl.thrift
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */

struct Property { }
(
   yarp.name="yarp::os::Property"
   yarp.includefile="yarp/os/Property.h"
)

/**
 * lineDetector_IDL
 *
 * IDL Interface to lineDetector services.
 */
service lineDetector_IDL
{
  /**
   * Start detection of start/finish line.
   * @param line string indicating the line to detect, start-line or finish-line.
   * @param timeout optional timeout in seconds, if > 0.
   * @return true on success/failure.
   */
   bool detect(1:string line, 2:i32 timeout=0)

   /**
    * Retrieve line.
    * @param line string indicating the line to detect, start-line or finish-line.
    * @return a property-like object in the form
    *         (pose_world (x y z ax ay az theta))
    *         (line_size (lenght width)).
    */
    Property get_line(1:string line);

   /**
    * Delete lines from opc and skeletonViewer.
    * @return true/false on success/failure.
    */
    bool reset();

   /**
    * Update robot's odometry with respect to specified line.
    * @param line_tag string indicating line with respect to calibrate, start-line or finish-line.
    * @param theta angle-coordinate to reach the specified line (degrees).
    * @return true/false on success/failure.
    */
    bool update_odometry(1:string line_tag, 2:double theta);

   /**
    * Navigate to the specified line.
    * @param line_tag string indicating line with respect to calibrate, start-line or finish-line.
    * @param theta angle-coordinate to reach the specified line (degrees).
    * @return true/false on success/failure.
    */
    bool go_to_line(1:string line_tag, 2:double theta);

   /**
    * Enables/Disables reuse of previous pose guess for current estimation
    * @param flag can be true or false
    * @return true/false on success/failure.
    */
    bool set_initial_guess(1:bool flag);

   /**
    * Checks if reuse of previous pose guess for current estimation is enabled
    * @return true if usage of initial guess is enables, false otherwise.
    */
    bool get_initial_guess();
}
