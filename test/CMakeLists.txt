# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

add_definitions(-D_USE_MATH_DEFINES)

add_executable(test-skeleton test-skeleton.cpp)
target_link_libraries(test-skeleton AssistiveRehab)
set_property(TARGET test-skeleton PROPERTY FOLDER "Tests")
add_test(NAME test-skeleton COMMAND test-skeleton)

add_executable(test-viewer test-viewer.cpp)
target_link_libraries(test-viewer AssistiveRehab)
set_property(TARGET test-viewer PROPERTY FOLDER "Tests")

add_executable(test-rom test-rom.cpp)
target_link_libraries(test-rom AssistiveRehab)
set_property(TARGET test-rom PROPERTY FOLDER "Tests")

add_executable(test-dtwlib test-dtwlib.cpp)
target_link_libraries(test-dtwlib AssistiveRehab)
set_property(TARGET test-dtwlib PROPERTY FOLDER "Tests")
