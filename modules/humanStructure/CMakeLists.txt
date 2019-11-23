# Copyright: (C) 2019 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(humanStructure)

message(STATUS "YARP_LIBRARIES = ${YARP_LIBRARIES}")
foreach(target ${YARP_LIBRARIES})
    get_property(x TARGET ${target} PROPERTY INTERFACE_LINK_LIBRARIES)
    message("    ${target} -> ${x}")
endforeach()

set(doc_files ${PROJECT_NAME}.xml)
source_group("DOC Files" FILES ${doc_files})

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} main.cpp ${doc_files})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
