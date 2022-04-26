# Copyright: (C) 2022 Istituto Italiano di Tecnologia
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Authors: Alexandre Antunes <alexandre.gomespereira@iit.it>

find_package(Gazebo)
find_package(GazeboYARPPlugins)

if(Gazebo_FOUND AND GazeboYARPPlugins_FOUND)
    project(forcePlugin)

    find_package(YCM ${YCM_REQUIRED_VERSION} REQUIRED)
    find_package(YARP ${YARP_REQUIRED_VERSION} REQUIRED)

    add_library(${PROJECT_NAME} SHARED src/forcePlugin.cc include/forcePlugin.hh )
    target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
    target_include_directories(${PROJECT_NAME} PRIVATE include  ${GAZEBO_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${GAZEBO_LIBRARIES} ${YARP_LIBRARIES} GazeboYARPPlugins::gazebo_yarp_singleton )
    install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)

endif()