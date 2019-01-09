/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_actionRecognizer_IDL
#define YARP_THRIFT_GENERATOR_actionRecognizer_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/sig/Matrix.h>

class actionRecognizer_IDL;


/**
 * actionRecognizer_IDL
 * IDL Interface to Action Recognizer services.
 */
class actionRecognizer_IDL : public yarp::os::Wire {
public:
  actionRecognizer_IDL();
  /**
   * Start the interaction.
   * @param number of frames used to train the model
   * @return true/false on success/failure.
   */
  virtual bool run(const std::int32_t nframes_);
  /**
   * Start the interaction.
   * @param tag of the current skeleton
   * @return true/false on success/failure.
   */
  virtual bool tags(const std::string& skel_tag_);
  /**
   * Load the name of the exercise to perform.
   * @param exercise name of the exercise to perform.
   * @return true/false on success/failure.
   */
  virtual bool load(const std::string& exercise);
  /**
   * Set the transformation matrix of the skeleton.
   * @param T_ is the transformation matrix.
   * @return true/false on success/failure.
   */
  virtual bool setTransformation(const yarp::sig::Matrix& T_);
  /**
   * Stop the interaction.
   * @return true/false on success/failure.
   */
  virtual bool stop();
  bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
