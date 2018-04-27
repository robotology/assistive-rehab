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
 * attentionManager_IDL
 *
 * IDL Interface to Attention Manager services.
 */
service attentionManager_IDL
{
   /**
    * Look at the specified skeleton.
    * @param tag the tag of the skeleton to look at.
    * @param keypoint the keypoint of the skeleton to look at.
    * @return true/false on success/failure.
    */
   bool look(1:string tag, 2:string keypoint="head");

   /**
    * Stop any ongoing actions.
    * @return true/false on success/failure.
    */
   bool stop();

   /**
    * Check if any action is being performed.
    * @return true/false on running/stationary.
    */
   bool is_running();

   /**
    * Enable autonomous mode.
    * @return true/false on success/failure.
    */
   bool set_auto();
}
