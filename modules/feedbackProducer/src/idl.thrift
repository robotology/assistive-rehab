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

struct Matrix { }
(
   yarp.name="yarp::sig::Matrix"
   yarp.includefile="yarp/sig/Matrix.h"
)

/**
 * feedbackProducer_IDL
 *
 * IDL Interface to alignment Manager services.
 */
service feedbackProducer_IDL
{

   /**
    * Set the tag of the template skeleton.
    * @param template_tag_ is the tag of the template.
    * @return true/false on success/failure.
    */
   bool setTemplateTag(1:string template_tag_);

   /**
    * Set the tag of the skeleton.
    * @param skel_tag_ is the tag of the skeleton.
    * @return true/false on success/failure.
    */
   bool setSkelTag(1:string skel_tag_);

   /**
    * Set the tag of the metric being analyzed.
    * @param metric_tag_ is the tag of the metric.
    * @return true/false on success/failure.
    */
   bool setMetric(1:string metric_tag_);

   /**
    * Set joints under analysis.
    * @param joint_list_ is the list of the joints under analysis.
    * @return true/false on success/failure.
    */
   bool setJoints(1:list<string> joint_list_);

   /**
    * Set the thresholds for the feedback.
    * @param feedback_thresholds_ is the matrix containing all the thresholds.
    * @return true/false on success/failure.
    */
   bool setFeedbackThresh(1:Matrix feedback_thresholds_);

   /**
    * Set the target to reach.
    * @param target_ is the vector containing target to reach.
    * @return true/false on success/failure.
    */
   bool setTarget(1:list<double> target_);

   /**
    * Set the transformation matrix of the skeleton.
    * @param T_ is the transformation matrix.
    * @return true/false on success/failure.
    */
   bool setTransformation(1:Matrix T_);

   /**
    * Start analysis.
    * @return true/false on success/failure.
    */
   bool start();

   /**
    * Stop analysis.
    * @return true/false on success/failure.
   */
   bool stop();

}
