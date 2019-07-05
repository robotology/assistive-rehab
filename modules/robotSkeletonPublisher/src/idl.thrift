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
 * robotSkeletonPublisher_IDL
 *
 * IDL Interface to Skeleton Player services.
 */
service robotSkeletonPublisher_IDL
{
   /**
    * Set visibility in the viewer.
    * @param flag true/false to set/unset visibility.
    * @return true/false on success/failure.
    */
   bool set_visibility(1:bool flag);

   /**
    * Set the name of the robot skeleton.
    * @param skeleton_name name of the robot skeleton.
    * @return true/false on success/failure.
    */
   bool set_robot_skeleton_name(1:string skeleton_name_);
}
