# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

find_package(IPOPT QUIET)
if(IPOPT_FOUND)
  project(skeletonRetriever)

  include_directories(${IPOPT_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src)
  add_executable(${PROJECT_NAME} src/utils.h src/main.cpp src/nlp.h src/nlp.cpp)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ${IPOPT_DEFINITIONS} _USE_MATH_DEFINES)
  set_property(TARGET ${PROJECT_NAME} APPEND_STRING PROPERTY LINK_FLAGS " ${IPOPT_LINK_FLAGS}")
  target_link_libraries(${PROJECT_NAME} ${IPOPT_LIBRARIES} ${YARP_LIBRARIES} ctrlLib AssistiveRehab)
  install(TARGETS ${PROJECT_NAME} DESTINATION bin)

  file(GLOB ini app/conf/*.ini)
  file(GLOB scripts app/scripts/*.template)
  yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
  yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
endif()
