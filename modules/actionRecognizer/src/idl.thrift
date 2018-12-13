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
 * actionRecognizer_IDL
 *
 * IDL Interface to Action Recognizer services.
 */
service actionRecognizer_IDL
{
   /**
    * Start the interaction.
    * @param number of frames used to train the model
    * @return true/false on success/failure.
    */
   bool run(1:i32 nframes_);

   /**
    * Start the interaction.
    * @param tag of the current skeleton
    * @return true/false on success/failure.
    */
   bool tags(1:string skel_tag_);

   /**
    * Load the name of the exercise to perform.
    * @param exercise name of the exercise to perform.
    * @return true/false on success/failure.
    */
   bool load(1:string exercise);

   /**
    * Set the transformation matrix of the skeleton.
    * @param T_ is the transformation matrix.
    * @return true/false on success/failure.
    */
   bool setTransformation(1:Matrix T_);

   /**
    * Stop the interaction.
    * @return true/false on success/failure.
    */
   bool stop();

}
