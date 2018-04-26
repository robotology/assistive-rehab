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
   * Load sequence from file.
   * @return true/false on success/failure.
   */
   bool loadSequence(1:string sequencer_file);

   /**
   * Load metric to analyze.
   * @param metric_tag name of the metric to analyze
   * @return true/false on success/failure.
   */
   bool loadMetric(1:string metric_tag);

}
