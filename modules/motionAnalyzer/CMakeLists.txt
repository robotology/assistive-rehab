# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Valentina Vasco <valentina.vasco@iit.it>

project(motionAnalyzer)

find_package(MATIO REQUIRED)
find_package(HDF5 REQUIRED)

include(YarpIDL)
yarp_add_idl(IDL_GEN_FILES src/idl.thrift)

file(GLOB source src/*.cpp)
file(GLOB header include/*.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})
source_group("IDL Files" FILES src/idl.thrift)
source_group("DOC Files" FILES ${PROJECT_NAME}.xml)

MESSAGE( STATUS "HDF5_INCLUDE_DIR: " ${HDF5_INCLUDE_DIR} )
MESSAGE( STATUS "HDF5_LIBRARIES: " ${HDF5_LIBRARIES} )

add_definitions(-D_USE_MATH_DEFINES)
include_directories(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${MATIO_INCLUDE_DIR} ${HDF5_INCLUDE_DIR})
add_executable(${PROJECT_NAME} ${source} ${header} src/idl.thrift ${IDL_GEN_FILES})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${HDF5_LIBRARIES} ${MATIO_LIBRARIES} AssistiveRehab)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
yarp_install(FILES ${PROJECT_NAME}.xml DESTINATION ${ICUBCONTRIB_MODULES_INSTALL_DIR})

add_subdirectory(app)
