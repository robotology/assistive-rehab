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

find_package(YCM)

# packages that shall come first as they include find_package(YARP)
# otherwise, we might have conflicts when selecting specific YARP components
find_package(ICUBcontrib REQUIRED)
find_package(ICUB REQUIRED)
find_package(CER)

find_package(YARP 3.3.103 REQUIRED COMPONENTS os sig dev math cv rosmsg)
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

include(InstallBasicPackageFiles)
add_subdirectory(lib)
include(AddUninstallTarget)

add_subdirectory(modules)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

add_subdirectory(app/gazebo/tug/plugins/tuginterface)

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
  foreach(_sh IN ITEMS ${sh})
    get_filename_component(_sh_name ${_sh} NAME)
    add_custom_command(TARGET copy_sh_in_build POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy ${_sh} ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/${_sh_name}
                       COMMENT "Copying ${_sh} to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/${_sh_name}")
  endforeach()
  install(PROGRAMS ${sh} DESTINATION bin)
endif()

file(GLOB js app/scripts/*.js)
add_custom_target(copy_js_in_build ALL)
foreach(_js IN ITEMS ${js})
  get_filename_component(_js_name ${_js} NAME)
  add_custom_command(TARGET copy_js_in_build POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy ${_js} ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/${_js_name}
                     COMMENT "Copying ${_js} to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}/${_js_name}")
endforeach()
install(PROGRAMS ${js} DESTINATION bin)
