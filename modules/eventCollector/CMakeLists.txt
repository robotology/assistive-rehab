# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Valentina Vasco <valentina.vasco@iit.it> Alexandre Antunes <alexandre.gomespereira@iit.it>

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
if(JSONCPP_FOUND)
    project(eventCollector)

    set(doc_files ${PROJECT_NAME}.xml)

    yarp_add_idl(IDL_GEN_FILES src/idl.thrift)

    file(GLOB source src/*.cpp)

    source_group("Source Files" FILES ${source})
    source_group("IDL Files" FILES src/idl.thrift)
    source_group("DOC Files" FILES ${doc_files})
    
    add_executable(${PROJECT_NAME} ${source} src/idl.thrift ${IDL_GEN_FILES} ${doc_files})
    target_include_directories(${PROJECT_NAME} PRIVATE ${JSONCPP_INCLUDE_DIR})
    target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
    target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} jsoncpp)
    install(TARGETS ${PROJECT_NAME} DESTINATION bin)

    file(GLOB json-config app/conf/*.json)
    yarp_install(FILES ${json-config} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})

endif()
