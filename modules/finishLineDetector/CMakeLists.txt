# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Valentina Vasco <valentina.vasco@iit.it>

project(finishLineDetector)

set(doc_files ${PROJECT_NAME}.xml)
source_group("DOC Files" FILES ${doc_files})

file(GLOB source src/*.cpp)
file(GLOB scripts app/scripts/*.template)

include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME} ${source} ${doc_files})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
