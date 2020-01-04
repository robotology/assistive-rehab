# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Ugo Pattacini <ugo.pattacini@iit.it>

cmake_minimum_required(VERSION 3.5)
project(AssistiveRehab LANGUAGES CXX C VERSION 0.4.0)

include(GNUInstallDirs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
  set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
if(UNIX)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/linux)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# packages that shall come first as they include find_package(YARP)
# otherwise, we might have conflicts when selecting specific YARP components
find_package(ICUBcontrib REQUIRED)
find_package(ICUB REQUIRED)
find_package(CER)

find_package(YARP 3.3.0 REQUIRED COMPONENTS os sig dev math cv rosmsg)
find_package(OpenCV REQUIRED)
find_package(VTK 8.1.0)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

find_package(IPOPT REQUIRED)

include(ICUBcontribOptions)
include(ICUBcontribHelpers)

option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" OFF)
option(BUILD_TESTING "Create tests using CMake" OFF)

if(BUILD_TESTING)
  enable_testing()
endif()

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
  endif()
endif()

icubcontrib_set_default_prefix()

add_subdirectory(lib)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME} VERSION ${${PROJECT_NAME}_VERSION}
                                            COMPATIBILITY AnyNewerVersion
                                            EXPORT ${PROJECT_NAME}
                                            VARS_PREFIX ${PROJECT_NAME}
                                            DEPENDENCIES "YARP REQUIRED" "ICUB REQUIRED" "OpenCV REQUIRED" "IPOPT REQUIRED"
                                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)
include(AddUninstallTarget)

add_subdirectory(modules)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

file(GLOB ini app/conf/*.ini)
file(GLOB xml app/conf/*.xml)
file(GLOB scripts app/scripts/*.template)
file(GLOB ipynb report/*.ipynb)
file(GLOB train_with_me_conf app/conf/train-with-me/*)
file(GLOB train_with_me_scripts app/scripts/train-with-me/*.template)
set(TRAIN_WITH_ME "train-with-me")
yarp_install(FILES ${ini} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${xml} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES ${ipynb} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME})
yarp_install(FILES ${train_with_me_conf} DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${PROJECT_NAME}/${TRAIN_WITH_ME})
yarp_install(FILES ${train_with_me_scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR}/${PROJECT_NAME}/${TRAIN_WITH_ME})

if(UNIX)
  file(GLOB sh app/scripts/*.sh report/*.sh)
  add_custom_target(copy_sh_in_build ALL)
  add_custom_command(TARGET copy_sh_in_build POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${sh} ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
                     COMMENT "Copying ${sh} to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/")
  install(PROGRAMS ${sh} DESTINATION bin)
endif()
