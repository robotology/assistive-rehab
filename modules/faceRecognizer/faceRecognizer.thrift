# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# faceRecognizer.thrift

/**
* faceRecognizer_IDL
*
* IDL Interface to \ref human structure module.
*/

service recognition_IDL
{
    /**
     * Train a face.
     * @param name: name of the label to assign.
     * @return true/false on success/failure.
     */
    bool train(1:string name);

    /**
     * Forget a face.
     * @param label: object name to forget. Use "all" to forget all faces.
     * @return true/false on success/failure.
     */
    bool forget(1:string label);

    /**
     * Set the confidenceThreshold.
     * @param thresh: threshold of the confidence.
     * @return true/false on success/failure.
     */
    bool setConfidenceThreshold(1:double thresh);

    /**
     * Get the confidenceThreshold.
     * @param thresh: threshold of the confidence.
     * @return value of confidence Threshold.
     */
    double getConfidenceThreshold();

    /**
     * Interaction Mode .
     * @param mode: use lift arm or closest face. Can be liftArm or closeFace, default is liftArm.
     * @return true/false on success/failure.
     */
    bool interactionMode(1:string mode);

    /**
     * Quit the module.
     * @return true/false on success/failure.
     */
    bool quit();
}
