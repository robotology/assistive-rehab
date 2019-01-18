# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Valentina Vasco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(feedbackSynthetizer)

set(doc_files ${PROJECT_NAME}.xml)
source_group("DOC Files" FILES ${doc_files})

add_executable(${PROJECT_NAME} src/main.cpp ${doc_files})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} AssistiveRehab ${FFTW3_LIBRARIES} ${GSL_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

file(GLOB scripts app/scripts/*.template)
file(GLOB ini app/conf/*.ini)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
