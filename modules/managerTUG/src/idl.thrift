/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file idl.thrift
 * @authors: Valentina Vasco
 */

/**
 * managerTUG_IDL
 *
 * IDL Interface to Manager Tug services.
 */
service managerTUG_IDL
{
   /**
    * Start the interaction.
    * @return true/false on success/failure.
    */
   bool start();

   /**
    * Stop the interaction.
    * @return true/false on success/failure.
    */
   bool stop();

   /**
    * Trigger speech.
    * @return true/false on success/failure.
    */
   bool trigger();

   /**
    * Set target to reach in simulation. Valid only if simulation is set to true.
    * @param x x-coordinate of the target to reach.
    * @param y y-coordinate of the target to reach.
    * @param theta theta angle of the target to reach.
    * @return true/false on success/failure.
    */
   bool set_target(1: double x=4.5, 2: double y=0.0, 3: double theta=0.0);

   /**
    * Get the time the user takes to complete the test.
    * @return measured time.
    */
   double get_measured_time();

   /**
    * Get the success status of the test.
    * @return passed / not_passed if the test was passed or not.
    */
   string get_success_status();

   /**
    * Returns true when the interaction has finished.
    * @return true / false if the interaction has finished or not.
    */
   bool has_finished();
}
