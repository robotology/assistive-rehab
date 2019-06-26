# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

project(robotSkeletonPublisher)
set(doc_files ${PROJECT_NAME}.xml)

source_group("DOC Files" FILES ${doc_files})

add_executable(${PROJECT_NAME} main.cpp ${doc_files})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} iKin CER::cer_kinematics AssistiveRehab)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
