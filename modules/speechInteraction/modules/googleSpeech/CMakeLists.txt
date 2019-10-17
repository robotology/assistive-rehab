# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Vadim Tikhanoff <vadim.tikhanoff@iit.it>

project(googleSpeech)

yarp_add_idl(IDL_GEN_FILES ${PROJECT_NAME}.thrift)

set(idl ${PROJECT_NAME}.thrift)
set(doc_files ${PROJECT_NAME}.xml)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})
source_group("IDL Files" FILES ${idl})
source_group("DOC Files" FILES ${doc})

add_executable(${PROJECT_NAME} main.cpp ${idl} ${IDL_GEN_FILES} ${doc} ${GRPC_SRCS} ${GRPC_HDRS} )

message("GRPC_INCLUDE_DIRS = ${GRPC_INCLUDE_DIRS}")
message("GRPC_LIBRARIES = ${GRPC_LIBRARIES}")
message("googleAPIsrc = ${googleAPIsrc}")

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${GRPC_LIBRARIES} googleAPI )
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

add_subdirectory(app)
