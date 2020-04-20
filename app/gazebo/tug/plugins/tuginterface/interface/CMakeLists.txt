# Copyright (C) 2015 iCub Facility
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

set(gazebo_assistiverehab_tuginterface_source src/tuginterface.cpp src/tugserver.cpp src/utils.cpp)
set(gazebo_assistiverehab_tuginterface_headers include/tuginterface.h include/tugserver.h include/utils.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${Boost_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(gazebo_assistiverehab_tuginterface SHARED
    ${gazebo_assistiverehab_tuginterface_source} ${gazebo_assistiverehab_tuginterface_headers})
target_compile_definitions(gazebo_assistiverehab_tuginterface PRIVATE _USE_MATH_DEFINES)

target_link_libraries(gazebo_assistiverehab_tuginterface GazeboYARPPlugins::gazebo_yarp_singleton
                                   ${YARP_LIBRARIES}
                                   ${GAZEBO_LIBRARIES}
                                   ${Boost_LIBRARIES}
                                   gazebo_assistiverehab_rpc_tuginterface)

yarp_install(TARGETS gazebo_assistiverehab_tuginterface
             COMPONENT Runtime
             LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
             ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR})
