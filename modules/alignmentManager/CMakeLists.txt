# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(FFTW3 QUIET)
if(FFTW3_FOUND)
    project(alignmentManager)

    add_definitions(-D_USE_MATH_DEFINES)

    file(GLOB source src/*.cpp)
    file(GLOB header include/*.h)

    source_group("Source Files" FILES ${source})
    source_group("Header Files" FILES ${header})

    message(STATUS "FFTW3: " ${FFTW3_INCLUDE_DIRS})

    include_directories(${PROJECT_SOURCE_DIR}/include)
    add_executable(${PROJECT_NAME} ${source} ${header})
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} AssistiveRehab ${FFTW3_LIBRARIES})
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)

    file(GLOB scripts app/scripts/*.template)
    yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
endif()
