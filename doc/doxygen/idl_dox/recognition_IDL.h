/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_recognition_IDL
#define YARP_THRIFT_GENERATOR_recognition_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class recognition_IDL;


/**
 * faceRecognizer_IDL
 * IDL Interface to \ref human structure module.
 */
class recognition_IDL : public yarp::os::Wire {
public:
  recognition_IDL();
  /**
   * Train a face.
   * @param name: name of the label to assign.
   * @return true/false on success/failure.
   */
  virtual bool train(const std::string& name);
  /**
   * Forget a face.
   * @param label: object name to forget. Use "all" to forget all faces.
   * @return true/false on success/failure.
   */
  virtual bool forget(const std::string& label);
  /**
   * Set the confidenceThreshold.
   * @param thresh: threshold of the confidence.
   * @return true/false on success/failure.
   */
  virtual bool setConfidenceThreshold(const double thresh);
  /**
   * Get the confidenceThreshold.
   * @param thresh: threshold of the confidence.
   * @return value of confidence Threshold.
   */
  virtual double getConfidenceThreshold();
  /**
   * Interaction Mode .
   * @param mode: use lift arm or closest face. Can be liftArm or closeFace, default is liftArm.
   * @return true/false on success/failure.
   */
  virtual bool interactionMode(const std::string& mode);
  /**
   * Quit the module.
   * @return true/false on success/failure.
   */
  virtual bool quit();
  bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
