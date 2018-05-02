# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift

/**
* motionAnalyzer_IDL
*
* IDL Interface to Motion Analyzer services.
*/

service motionAnalyzer_IDL
{

   /**
   * Load motion repertoire from file.
   * @return true/false on success/failure.
   */
   bool loadMotionList();

   /**
   * Load metric to analyze.
   * @param metric_tag name of the metric to analyze
   * @return session duration in seconds / -1 on failure.
   */
   double loadMetric(1:string metric_tag);

   /**
   * List available metrics.
   * @return the list of the available metrics as defined in the motion-repertoire.
   */
   list<string> listMetrics();

   /**
   * Start processing.
   * @return true/false on success/failure.
   */
   bool start();

   /**
   * Stop processing.
   * @return true/false on success/failure.
   */
   bool stop();

   /**
   * Select skeleton by its tag.
   * @param skel_tag tag of the skeleton to process
   * @return true/false on success/failure.
   */
   bool selectSkel(1:string skel_tag);

   /**
   * Get the quality of the performed exercise.
   * @return the quality of the performed exercise between 0 and 1.
   */
   double getQuality();

}
