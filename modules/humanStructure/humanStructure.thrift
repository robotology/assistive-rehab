# Copyright: (C) 2019 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# humanStructure.thrift

/**
* humanStructure_IDLServer
*
* IDL Interface to \ref human structure module.
*/
service humanStructure_IDLServer
{
    /**
     * Set the threshold number for finding the arm.
     * @return true/false on success/failure
     */
    bool setArmThresh(1:i32 threshold );
    
    /**
     * Get threshold value for finding the arm.
     * @return number of frames
     */
    i32 getArmThresh();

    /**
     * Quit the module.
     * @return true/false on success/failure.
     */
    bool quit();
}
