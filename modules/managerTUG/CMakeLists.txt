# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Valentina Vasco <valentina.vasco@iit.it>

project(managerTUG)

set(doc_files ${PROJECT_NAME}.xml)

yarp_add_idl(IDL_GEN_FILES src/idl.thrift)

source_group("IDL Files" FILES src/idl.thrift)
source_group("DOC Files" FILES ${doc_files})

add_executable(${PROJECT_NAME} src/main.cpp src/AnswerManager.h src/AnswerManager.cpp src/idl.thrift ${IDL_GEN_FILES} ${doc_files})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} AssistiveRehab)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

file(GLOB ini app/conf/*.ini)
yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})

