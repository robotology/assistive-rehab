# Copyright (C) 2015 iCub Facility
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

find_package(Gazebo)
find_package(GazeboYARPPlugins)

if(Gazebo_FOUND AND GazeboYARPPlugins_FOUND)
    # Add Gazebo CXX flags, to support Gazebo 6 reckless dependency on C++11
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

    add_subdirectory(interface)
    add_subdirectory(thrift)
endif()

