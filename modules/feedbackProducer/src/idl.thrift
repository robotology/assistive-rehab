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
    * Set the thresholds for the feedback.
    * @param feedback_thresholds_ is the matrix containing all the thresholds.
    * @return true/false on success/failure.
    */
   bool setFeedbackThresh(1:Matrix feedback_thresholds_);

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
