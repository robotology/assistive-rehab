# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

set(${PROJECT_NAME}_SRC src/helpers.cpp
                        src/skeleton.cpp
                        src/dtw.cpp)

set(${PROJECT_NAME}_HDR include/AssistiveRehab/helpers.h
                        include/AssistiveRehab/skeleton.h
                        include/AssistiveRehab/dtw.h)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC} ${${PROJECT_NAME}_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION}
                                      PUBLIC_HEADER "${${PROJECT_NAME}_HDR}")

target_link_libraries(${PROJECT_NAME} PUBLIC ${YARP_LIBRARIES} PRIVATE ${OpenCV_LIBS})

target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

install(TARGETS ${PROJECT_NAME}
        EXPORT  ${PROJECT_NAME}
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                     COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                     COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}" COMPONENT dev)

install_basic_package_files(${PROJECT_NAME} VERSION ${${PROJECT_NAME}_VERSION}
                                            COMPATIBILITY AnyNewerVersion
                                            EXPORT ${PROJECT_NAME}
                                            VARS_PREFIX ${PROJECT_NAME}
                                            DEPENDENCIES "YARP REQUIRED"
                                            PRIVATE_DEPENDENCIES "ICUB REQUIRED"
                                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)

