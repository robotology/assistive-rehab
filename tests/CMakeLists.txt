# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

add_definitions(-D_USE_MATH_DEFINES)

add_executable(test-skeleton test-skeleton.cpp)
target_link_libraries(test-skeleton ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-skeleton PROPERTY FOLDER "Tests")
add_test(NAME test-skeleton COMMAND test-skeleton)

add_executable(test-viewer test-viewer.cpp)
target_link_libraries(test-viewer ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-viewer PROPERTY FOLDER "Tests")

add_executable(test-rom test-rom.cpp)
target_link_libraries(test-rom ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-rom PROPERTY FOLDER "Tests")

add_executable(test-dtwlib test-dtwlib.cpp)
target_link_libraries(test-dtwlib ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-dtwlib PROPERTY FOLDER "Tests")

find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIRS})
  add_executable(test-depth-overlay test-depth-overlay.cpp)
  target_link_libraries(test-depth-overlay ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
  set_property(TARGET test-depth-overlay PROPERTY FOLDER "Tests")
endif()
