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
    * Start the interaction with occlusion.
    * @return true/false on success/failure.
    */
   bool start_occlusion();

}
