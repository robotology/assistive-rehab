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
    * @return true/false on success/failure.
    */
   bool run();

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
