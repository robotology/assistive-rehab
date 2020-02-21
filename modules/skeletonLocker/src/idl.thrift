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

/**
 * skeletonLocker_IDL
 *
 * IDL Interface to Skeleton Locker services.
 */
service skeletonLocker_IDL
{
   /**
    * Set the tag of the skeleton to lock.
    * @param tag of the skeleton to lock.
    * @return true/false on success/failure.
    */
   bool set_skeleton_tag(1:string tag)
}
