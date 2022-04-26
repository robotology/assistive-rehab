/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file idl.thrift
 * @authors: Valentina Vasco <valentina.vasco@iit.it>
 */


/**
 * obstacleDetector_IDL
 *
 * IDL Interface to Obstacle Detector services.
 */
service obstacleDetector_IDL
{
   /**
    * Returns the time taken to trigger a stop signal if an obstacle is detected
    * @return the time taken to trigger a stop signal if an obstacle is detected.
    */
   double get_time_to_stop();

   /**
    * Start obstacle detection.
    * @return true/false on success/failure.
    */
   bool start();

   /**
    * Stop obstacle detection.
    * @return true/false on success/failure.
    */
   bool stop();
}
