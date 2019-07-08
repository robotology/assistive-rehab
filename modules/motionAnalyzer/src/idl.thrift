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
   * @return true/false on failure.
   */
   bool loadMetric(1:string metric_tag);

   /**
   * Get the type of motion.
   * @return string containing the type of motion / empty string on failure.
   */
   string getMotionType();

   /**
   * List available metrics.
   * @return the list of the available metrics as defined in the motion-repertoire.
   */
   list<string> listMetrics();

   /**
   * Start processing.
   * @param use_robot_template true if robot template is used.
   * @return true/false on success/failure.
   */
   bool start(1:bool use_robot_template);

   /**
   * Stop feedback.
   * @return true/false on success/failure.
   */
   bool stop_feedback();

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
   * List joints on which feedback is computed.
   * @return the list of joints on which feedback is computed.
   */
   list<string> listJoints()

   /**
   * Select the part to move.
   * @return true/false on success/failure.
   */
   bool setPart(1:string part);

   /**
   * Select the template for the analysis.
   * @return true/false on success/failure.
   */
   bool setTemplateTag(1:string template_tag);

   /**
   * Mirror the skeleton template.
   * @param robot_skeleton_mirror if true, robot template has to be mirrored.
   * @return true/false on success/failure.
   */
   bool mirrorTemplate(1:bool robot_skeleton_mirror)

}
