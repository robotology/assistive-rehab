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
 * skeletonPlayer_IDL
 *
 * IDL Interface to Skeleton Player services.
 */
service skeletonPlayer_IDL
{
   /**
    * Load skeleton file.
    * @param file the name of the file containing the skeleton data. 
    * @param context the context used to search for the file.
    * @return true/false on success/failure.
    */
   bool load(1:string file="skeleton.log", 2:string context="skeletonPlayer");

   /**
    * Start streaming skeleton data.
    * @param n_sessions number of repetitions (0 means infinite).
    * @param t_warp specifies the warping factor squeezing (dilating) in time the original stream if <1 (>1).
    * @param t_begin specifies the stream starting time computed from the time origin.
    * @param t_end specifies the stream ending time computed from the stream end. 
    * @return true/false on success/failure.
    */
   bool start(1:i32 n_sessions=1, 2:double t_warp=1.0, 3:double t_begin=0.0, 4:double t_end=0.0);

   /**
    * Stop any ongoing streaming.
    * @return true/false on success/failure.
    */
   bool stop();

   /**
    * Check if the skeleton is being streamed out.
    * @return true/false on running/stationary.
    */
   bool is_running();

   /**
    * Put in opc a specified skeleton frame.
    * @param b_begin specifies the time computed from the origin whose frame is to be put in opc.
    * @return true/false on success/failure.
    */
   bool put_in_opc(1:double t_begin=0.0);

   /**
    * Remove from opc any skeleton frame.
    * @return true/false on success/failure.
    */
   bool remove_from_opc();

   /**
    * Rename skeleton.
    * @param new_tag the new tag of the skeleton.
    * @return true/false on success/failure.
    */
   bool set_tag(1:string new_tag);   

   /**
    * Set opacity.
    * @param new_opacity the new opacity of the skeleton.
    * @return true/false on success/failure.
    */
   bool set_opacity(1:double new_opacity);   
}
