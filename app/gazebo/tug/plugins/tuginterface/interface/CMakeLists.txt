# Copyright (C) 2015 iCub Facility
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

set(${PROJECT_NAME}_source src/tuginterface.cpp src/tugserver.cpp)
set(${PROJECT_NAME}_headers include/tuginterface.h include/tugserver.h)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${Boost_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_source} ${${PROJECT_NAME}_headers})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

target_link_libraries(${PROJECT_NAME} GazeboYARPPlugins::gazebo_yarp_singleton
                                   ${YARP_LIBRARIES}
                                   ${GAZEBO_LIBRARIES}
                                   ${Boost_LIBRARIES}
                                   gazebo_assistiverehab_rpc_tuginterface)

yarp_install(TARGETS ${PROJECT_NAME}
             COMPONENT Runtime
             LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
             ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR})
