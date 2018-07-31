# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl_rom.thrift

/**
* rom-test_IDL
*
* IDL Interface to test-rom.
*/

service rom-test_IDL
{

   /**
   * Set rom to visualise.
   * @return true/false on success/failure.
   */
   bool setRom();

}
