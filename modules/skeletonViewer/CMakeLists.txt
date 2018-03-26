# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

find_package(VTK QUIET)
if(VTK_FOUND)
  project(skeletonViewer)

  include(${VTK_USE_FILE})
  add_definitions(-D_USE_MATH_DEFINES)
  include_directories(${YARP_INCLUDE_DIRS})
  add_executable(${PROJECT_NAME} main.cpp)
  target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ${VTK_LIBRARIES} AssistiveRehab)
  install(TARGETS ${PROJECT_NAME} DESTINATION bin)
endif()

