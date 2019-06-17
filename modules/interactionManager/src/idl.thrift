/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file idl.thrift
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

/**
 * interactionManager_IDL
 *
 * IDL Interface to Interaction Manager services.
 */
service interactionManager_IDL
{
   /**
    * Start the interaction with raising hand.
    * @return true/false on success/failure.
    */
   bool start_with_hand();

   /**
    * Stop the interaction.
    * @return true/false on success/failure.
    */
   bool stop();

   /**
    * Start imitation phase with occlusion.
    * @return true/false on success/failure.
    */
   bool start_occlusion();

  /**
   * Start observation phase.
   * @return true/false on success/failure.
   */
   bool start_observation();

  /**
   * Start imitation phase.
   * @return true/false on success/failure.
   */
   bool start_imitation();

}
