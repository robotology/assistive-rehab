# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Vadim Tikhanoff <vadim.tikhanoff@iit.it>
#
# googleSpeech.thrift

/**
* googleSpeech_IDL
*
* IDL Interface to google speech cloud services 
*/

service googleSpeech_IDL
{

    /**
     * Starts the sound acquisition
     * @return true/false on success/failure
     */
    bool start();

    /**
     * Stops the sound acquisition
     * @return true/false on success/failure
     */
    bool stop();

    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
