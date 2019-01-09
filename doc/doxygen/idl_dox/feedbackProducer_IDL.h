/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_feedbackProducer_IDL
#define YARP_THRIFT_GENERATOR_feedbackProducer_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/sig/Matrix.h>

class feedbackProducer_IDL;


/**
 * feedbackProducer_IDL
 * IDL Interface to alignment Manager services.
 */
class feedbackProducer_IDL : public yarp::os::Wire {
public:
  feedbackProducer_IDL();
  /**
   * Set the tag of the template skeleton.
   * @param template_tag_ is the tag of the template.
   * @return true/false on success/failure.
   */
  virtual bool setTemplateTag(const std::string& template_tag_);
  /**
   * Set the tag of the skeleton.
   * @param skel_tag_ is the tag of the skeleton.
   * @return true/false on success/failure.
   */
  virtual bool setSkelTag(const std::string& skel_tag_);
  /**
   * Set the tag of the metric being analyzed.
   * @param metric_tag_ is the tag of the metric.
   * @return true/false on success/failure.
   */
  virtual bool setMetric(const std::string& metric_tag_);
  /**
   * Set joints under analysis.
   * @param joint_list_ is the list of the joints under analysis.
   * @return true/false on success/failure.
   */
  virtual bool setJoints(const std::vector<std::string> & joint_list_);
  /**
   * Set the thresholds for the feedback.
   * @param feedback_thresholds_ is the matrix containing all the thresholds.
   * @return true/false on success/failure.
   */
  virtual bool setFeedbackThresh(const yarp::sig::Matrix& feedback_thresholds_);
  /**
   * Set the target to reach.
   * @param target_ is the vector containing target to reach.
   * @return true/false on success/failure.
   */
  virtual bool setTarget(const std::vector<double> & target_);
  /**
   * Set the transformation matrix of the skeleton.
   * @param T_ is the transformation matrix.
   * @return true/false on success/failure.
   */
  virtual bool setTransformation(const yarp::sig::Matrix& T_);
  /**
   * Start analysis.
   * @return true/false on success/failure.
   */
  virtual bool start();
  /**
   * Stop analysis.
   * @return true/false on success/failure.
   */
  virtual bool stop();
  bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
