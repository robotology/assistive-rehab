/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file idl.thrift
 * @authors: Valentina Vasco <valentina.vasco@iit.it> Alexandre Antunes <alexandre.gomespereira@iit.it>
 */


/**
 * eventCollector_IDL
 *
 * IDL Interface to eventCollector services.
 */
service eventCollector_IDL
{
   /**
   * Start collecting.
   * @return true/false on success/failure.
   */
   bool start();

   /**
   * Stop collecting.
   * @return true/false on success/failure.
   */
   bool stop();

   /**
   * Save collected data to JSON file.
   * @return true/false on success/failure.
   */
   bool save_data();
}
