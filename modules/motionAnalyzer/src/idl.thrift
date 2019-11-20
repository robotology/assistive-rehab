/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
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
 * motionAnalyzer_IDL
 *
 * IDL Interface to Motion Analyzer services.
 */
service motionAnalyzer_IDL
{
   /**
   * Load exercise to analyze.
   * @param exercise_tag name of the exercise to analyze
   * @return true/false on failure.
   */
   bool loadExercise(1:string exercise_tag);

   /**
   * Get the name of the exercise begin performed.
   * @return string containing the name of the exercise begin performed / empty string on failure.
   */
   string getExercise();

   /**
   * List available exercises.
   * @return the list of the available exercises as defined in the motion-repertoire.
   */
   list<string> listExercises();

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
   bool stopFeedback();

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
   * Select property to visualize.
   * @param prop property to visualize
   * @return true/false on success/failure.
   */
   bool selectMetricProp(1:string prop_tag);

   /**
   * List the available properties computable for the current metric.
   * @return the list of the available properties computable for the current metric.
   */
   list<string> listMetricProps()

   /**
   * Select metric to analyze.
   * @param metric_tag metric to analyze
   * @return true/false on success/failure.
   */
   bool selectMetric(1:string metric_tag);

   /**
   * List the available metrics for the current exercise.
   * @return the list of the available metrics for the current exercise.
   */
   list<string> listMetrics()

   /**
   * Get the metric to visualise.
   * @return metric to visualise.
   */
   string getCurrMetricProp();

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
   bool mirrorTemplate(1:bool robot_skeleton_mirror);

   /**
   * Detect if the person is standing.
   * @param standing_thresh threshold on the speed of the shoulder center height [m/s].
   * @return true/false if the person is/is not standing.
   */
   bool isStanding(1:double standing_thresh);

   /**
   * Detect if the person is sitting.
   * @param standing_thresh threshold on the speed of the shoulder center height [m/s].
   * @return true/false if the person is/is not standing.
   */
   bool isSitting(1:double standing_thresh);

   /**
   * Detect if the finish line has been crossed.
   * @param finishline_thresh distance between foot and finish line [m].
   * @return true/false if the finish line has been/not been crossed.
   */
   bool hasCrossedFinishLine(1:double finishline_thresh);

   /**
   * Set the pose of the finish line.
   * @param pose of the finish line.
   * @return true/false on success/failure.
   */
   bool setLinePose(1:list<double> line_pose);
}
