# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(OpenCV QUIET)
if(OpenCV_FOUND)
    project(human-structure)

    include_directories(${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME} main.cpp)
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)

    add_subdirectory(app)
endif()