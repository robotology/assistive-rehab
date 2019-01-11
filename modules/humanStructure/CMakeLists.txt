# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(humanStructure)

set(doc_files ${PROJECT_NAME}.xml)
source_group("DOC Files" FILES ${doc_files})

add_executable(${PROJECT_NAME} main.cpp ${doc_files})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

add_subdirectory(app)
