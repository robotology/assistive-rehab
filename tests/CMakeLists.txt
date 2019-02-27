# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

add_executable(test-skeleton test-skeleton.cpp)
target_compile_definitions(test-skeleton PRIVATE _USE_MATH_DEFINES)
target_link_libraries(test-skeleton ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-skeleton PROPERTY FOLDER "Tests")
add_test(NAME test-skeleton COMMAND test-skeleton)

add_executable(test-viewer test-viewer.cpp)
target_compile_definitions(test-viewer PRIVATE _USE_MATH_DEFINES)
target_link_libraries(test-viewer ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-viewer PROPERTY FOLDER "Tests")

add_executable(test-rom test-rom.cpp)
target_compile_definitions(test-rom PRIVATE _USE_MATH_DEFINES)
target_link_libraries(test-rom ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-rom PROPERTY FOLDER "Tests")

add_executable(test-dtwlib test-dtwlib.cpp)
target_compile_definitions(test-dtwlib PRIVATE _USE_MATH_DEFINES)
target_link_libraries(test-dtwlib ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-dtwlib PROPERTY FOLDER "Tests")

add_executable(test-overlay test-overlay.cpp)
target_link_libraries(test-overlay ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-overlay PROPERTY FOLDER "Tests")

add_executable(test-tracker test-tracker.cpp)
target_compile_definitions(test-tracker PRIVATE _USE_MATH_DEFINES)
target_link_libraries(test-tracker ${YARP_LIBRARIES} AssistiveRehab)
set_property(TARGET test-tracker PROPERTY FOLDER "Tests")
