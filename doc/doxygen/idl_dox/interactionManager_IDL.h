/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_interactionManager_IDL
#define YARP_THRIFT_GENERATOR_interactionManager_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class interactionManager_IDL;


/**
 * interactionManager_IDL
 * IDL Interface to Interaction Manager services.
 */
class interactionManager_IDL : public yarp::os::Wire {
public:
  interactionManager_IDL();
  /**
   * Start the interaction.
   * @return true/false on success/failure.
   */
  virtual bool start();
  /**
   * Stop the interaction.
   * @return true/false on success/failure.
   */
  virtual bool stop();
  bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
