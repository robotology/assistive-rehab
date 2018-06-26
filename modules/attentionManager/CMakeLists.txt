# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

project(attentionManager)

yarp_add_idl(IDL_GEN_FILES src/idl.thrift)

source_group("IDL Files" FILES src/idl.thrift)

add_definitions(-D_USE_MATH_DEFINES)
add_executable(${PROJECT_NAME} src/main.cpp src/idl.thrift ${IDL_GEN_FILES})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} AssistiveRehab)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

file(GLOB scripts app/scripts/*.template)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
