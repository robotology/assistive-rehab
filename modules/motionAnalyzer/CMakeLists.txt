# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Valentina Vasco <valentina.vasco@iit.it>

find_package(MATIO QUIET)
find_package(HDF5 QUIET)
if(MATIO_FOUND AND HDF5_FOUND)
    project(motionAnalyzer)

    yarp_add_idl(IDL_GEN_FILES src/idl.thrift)

    file(GLOB source src/*.cpp)
    file(GLOB header include/*.h)

    source_group("Source Files" FILES ${source})
    source_group("Header Files" FILES ${header})
    source_group("IDL Files" FILES src/idl.thrift)
    source_group("DOC Files" FILES ${PROJECT_NAME}.xml)

    message(STATUS "HDF5_INCLUDE_DIR: " ${HDF5_INCLUDE_DIR})
    message(STATUS "HDF5_LIBRARIES: " ${HDF5_LIBRARIES})

    add_definitions(-D_USE_MATH_DEFINES)
    include_directories(${PROJECT_SOURCE_DIR}/include ${MATIO_INCLUDE_DIR} ${HDF5_INCLUDE_DIR})
    add_executable(${PROJECT_NAME} ${source} ${header} src/idl.thrift ${IDL_GEN_FILES})
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${HDF5_LIBRARIES} ${MATIO_LIBRARIES} AssistiveRehab)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)

    file(GLOB log app/conf/*.log)
    file(GLOB ini app/conf/*.ini)  
    file(GLOB scripts app/scripts/*.template)
    yarp_install(FILES ${log} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
    yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
    yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
endif()
