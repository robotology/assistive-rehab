/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_skeletonPlayer_IDL
#define YARP_THRIFT_GENERATOR_skeletonPlayer_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/sig/Matrix.h>

class skeletonPlayer_IDL;


/**
 * skeletonPlayer_IDL
 * IDL Interface to Skeleton Player services.
 */
class skeletonPlayer_IDL : public yarp::os::Wire {
public:
  skeletonPlayer_IDL();
  /**
   * Load skeleton file.
   * @param file the name of the file containing the skeleton data.
   * @param context the context used to search for the file.
   * @return true/false on success/failure.
   */
  virtual bool load(const std::string& file = "skeleton.log", const std::string& context = "skeletonPlayer");
  /**
   * Start streaming skeleton data.
   * @param n_sessions number of repetitions (0 means infinite).
   * @param t_warp specifies the warping factor squeezing (dilating) in time the original stream if <1 (>1).
   * @param t_begin specifies the stream starting time computed from the time origin.
   * @param t_end specifies the stream ending time computed from the stream end.
   * @return true/false on success/failure.
   */
  virtual bool start(const std::int32_t n_sessions = 1, const double t_warp = 1, const double t_begin = 0, const double t_end = 0);
  /**
   * Stop any ongoing streaming.
   * @return true/false on success/failure.
   */
  virtual bool stop();
  /**
   * Check if the skeleton is being streamed out.
   * @return true/false on running/stationary.
   */
  virtual bool is_running();
  /**
   * Put in opc a specified skeleton frame.
   * @param t_begin specifies the time computed from the origin whose frame is to be put in opc.
   * @return true/false on success/failure.
   */
  virtual bool put_in_opc(const double t_begin = 0);
  /**
   * Remove from opc any skeleton frame.
   * @return true/false on success/failure.
   */
  virtual bool remove_from_opc();
  /**
   * Rename skeleton.
   * @param new_tag the new tag of the skeleton.
   * @return true/false on success/failure.
   */
  virtual bool set_tag(const std::string& new_tag);
  /**
   * * Retrieve skeleton maximum path.
   * * @param t_begin specifies the time computed from the origin whose frame will be used
   *          to compute the maxium path. If t_begin<0, then an average is performed over
   *          the whole set of skeletons.
   * * @return the path.
   */
  virtual double get_maxpath(const double t_begin = -1);
  /**
   * Normalize skeleton.
   * @return true/false on success/failure.
   */
  virtual bool normalize();
  /**
   * Rescale skeleton.
   * @param s the scale.
   * @return true/false on success/failure.
   */
  virtual bool scale(const double s);
  /**
   * Apply homogeneous transformation to the skeleton.
   * @param T is the 4x4 homogeneous matrix.
   * @return true/false on success/failure.
   */
  virtual bool move(const yarp::sig::Matrix& T);
  /**
   * Set opacity.
   * @param new_opacity the new opacity of the skeleton.
   * @return true/false on success/failure.
   */
  virtual bool set_opacity(const double new_opacity);
  /**
   * Set color.
   * @param new_r the red channel in [0,1].
   * @param new_g the green channel in [0,1].
   * @param new_b the blue channel in [0,1].
   * @return true/false on success/failure.
   */
  virtual bool set_color(const double new_r, const double new_g, const double new_b);
  bool read(yarp::os::ConnectionReader& connection) override;
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
