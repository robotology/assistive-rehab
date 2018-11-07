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
   bool run(1:i32 nsteps_);

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
    * Stop the interaction.
    * @return true/false on success/failure.
    */
   bool stop();

}
