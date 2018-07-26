# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(FFTW3 QUIET)
find_package(GSL QUIET)
if(FFTW3_FOUND AND GSL_FOUND)
    project(alignmentManager)

    add_definitions(-D_USE_MATH_DEFINES)
    message(STATUS "FFTW3: " ${FFTW3_INCLUDE_DIRS})
    message(STATUS "GSL: " ${GSL_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME} src/main.cpp)
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} AssistiveRehab ${FFTW3_LIBRARIES} ${GSL_LIBRARIES} ctrlLib)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)
    file(GLOB scripts app/scripts/*.template)
    yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
endif()
