#=============================================================================
# Copyright 2018  iCub Facility, Istituto Italiano di Tecnologia
#   Authors: Vadim Tikhanoff <vadim.tikhanoff@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of YCM, substitute the full
#  License text for the above reference.)

find_path(googleapis_INCLUDE_DIR
          NAMES google/api/http.pb.h
          PATHS ${GOOGLEAPIS_GENS_PATH}
                ENV GOOGLEAPIS_GENS_PATH)


message("googleapis_INCLUDE_DIR = ${googleapis_INCLUDE_DIR}")

file(GLOB google_api ${googleapis_INCLUDE_DIR}/google/api/*.pb.cc)
file(GLOB google_rpc ${googleapis_INCLUDE_DIR}/google/rpc/*.pb.cc)
file(GLOB google_speech ${googleapis_INCLUDE_DIR}/google/cloud/speech/v1/*.pb.cc)
file(GLOB google_longrunning ${googleapis_INCLUDE_DIR}/google/longrunning/*.pb.cc)
file(GLOB google_protobuf ${googleapis_INCLUDE_DIR}/google/protobuf/*.pb.cc)
file(GLOB google_language ${googleapis_INCLUDE_DIR}/google/cloud/language/v1/*.pb.cc)

set(googleAPIsrc    ${google_api}
		    ${google_rpc}
		    ${google_speech}
		    ${google_longrunning}
		    ${google_protobuf}
		    ${google_language})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(googleapis REQUIRED_VARS googleapis_INCLUDE_DIR
                                                           googleAPIsrc)

# Set package properties if FeatureSummary was included
if(COMMAND set_package_properties)
    set_package_properties(googleapis PROPERTIES DESCRIPTION "googleapis" URL "https://github.com/googleapis/googleapis")
endif()
