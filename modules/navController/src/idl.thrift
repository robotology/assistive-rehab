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

/**
 * navController_IDL
 *
 * IDL Interface to Navigation Controller services.
 */
service navController_IDL
{
   /**
    * Start navigation while controlling distance from the specified skeleton.
    * @param skeleton_tag is the skeleton's tag.
    * @return true/false on success/failure.
    */
   bool start(1:string skeleton_tag);

   /**
    * Stop navigation.
    * @return true/false on success/failure.
    */
   bool stop();

   /**
    * Query which skeleton is currently under control.
    * @return the name of the skeleton; empty otherwise.
    */
   string which_skeleton();
}
