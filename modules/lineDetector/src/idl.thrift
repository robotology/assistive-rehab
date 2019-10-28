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

struct Vector { }
(
   yarp.name="yarp::sig::Vector"
   yarp.includefile="yarp/sig/Vector.h"
)

struct Matrix { }
(
   yarp.name="yarp::sig::Matrix"
   yarp.includefile="yarp/sig/Matrix.h"
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
    * Get the pose of the line.
    * @param line string indicating the line to detect, start-line or finish-line.
    * @return pose of the line with respect to the world frame (start-line).
    */
    Vector get_line_pose(1:string line);

   /**
    * Reset lines.
    * @return true/false on success/failure.
    */
    bool reset();

   /**
    * Stop detection.
    * @return true/false on success/failure.
    */
    bool stop();

}
