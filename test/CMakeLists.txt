# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

add_definitions(-D_USE_MATH_DEFINES)
add_executable(test-skeleton test-skeleton.cpp)
add_executable(test-viewer test-viewer.cpp)
add_executable(test-rom test-rom.cpp)
target_link_libraries(test-skeleton AssistiveRehab)
target_link_libraries(test-viewer AssistiveRehab)
target_link_libraries(test-rom AssistiveRehab)
add_test(NAME test-skeleton COMMAND test-skeleton)
