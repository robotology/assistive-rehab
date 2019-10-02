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
* FollowedSkeletonInfo
*
* IDL structure to send info on followed skeleton.
*/
struct FollowedSkeletonInfo
{
   /**
   * the tag of the followed skeleton; empty otherwise.
   */
   1:string tag;

   /**
   * the x-coordinate.
   */
   2:double x;

   /**
   * the y-coordinate.
   */
   3:double y;

   /**
   * the z-coordinate.
   */
   4:double z;
}

struct Vector { }
(
   yarp.name="yarp::sig::Vector"
   yarp.includefile="yarp/sig/Vector.h"
)

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
    * Check if the robot is following a skeleton.
    * @return info on the followed skeleton in \ref FollowedSkeletonInfo format.
    */
   FollowedSkeletonInfo is_following();

   /**
    * Check if any skeleton is with one hand raised.
    * @return the list of skeletons' tags.
    */
   list<string> is_any_raised_hand();

   /**
    * Check if the specified skeleton is with one hand raised.
    * @param tag the tag of the skeleton.
    * @return true/false on success/failure.
    */
   bool is_with_raised_hand(1:string tag);

   /**
    * Enable autonomous mode.
    * @param seek_for_finish_line set this to true for TUG.
    * @return true/false on success/failure.
    */
   bool set_auto(1:bool seek_for_finish_line=false);

   /**
    * Enable virtual scenario mode.
    * @return true/false on success/failure.
    */
   bool set_virtual();

   /**
    * Set the name of the robot skeleton to avoid following it.
    * @param robot_skeleton_name name of the robot skeleton.
    * @return true/false on success/failure.
    */
   bool set_robot_skeleton_name(1:string robot_skeleton_name);

   /**
    * Get the pose of the finish line.
    * @return pose of the finish line with respect to the camera.
    */
    Vector get_line_pose();

}
