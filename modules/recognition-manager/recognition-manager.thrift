# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# recognition-manager.thrift

/**
* recognition-manager_IDL
*
* IDL Interface to \ref human structure module.
*/

service recognition_IDL
{
    /**
     * Set the lower bound of the colour threshold
     * @param name: name of the label to assign
     * @return true/false on success/failure
     */
    bool train(1:string name);

    /**
     * Set the lower bound of the colour threshold
     * @param label: object name to forget
     * @return true/false on success/failure
     */
    bool forget(1:string label);

    /**
     * Set the confidenceThreshold
     * @param thresh: threshold of the confidence
     * @return true/false on success/failure
     */
    bool setConfidenceThreshold(1:double thresh);

    /**
     * Get the confidenceThreshold
     * @param thresh: threshold of the confidence
     * @return value of confidence Threshold
     */
    double getConfidenceThreshold();

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
