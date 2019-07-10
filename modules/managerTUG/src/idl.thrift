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

}
