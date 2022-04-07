# Copyright: (C) 2021 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Alexandre Antunes
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(contactPlugin)

find_package(YCM ${YCM_REQUIRED_VERSION} REQUIRED)
find_package(YARP ${YARP_REQUIRED_VERSION} REQUIRED)
find_package(gazebo REQUIRED)
find_package(GazeboYARPPlugins REQUIRED)

add_library(${PROJECT_NAME} SHARED src/contactPlugin.cc include/contactPlugin.hh )
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_include_directories(${PROJECT_NAME} PRIVATE include  ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${GAZEBO_LIBRARIES} ${YARP_LIBRARIES} GazeboYARPPlugins::gazebo_yarp_singleton )
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)