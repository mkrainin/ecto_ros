cmake_minimum_required(VERSION 2.8)
project(ecto_ros)
message(STATUS "--- ${PROJECT_NAME} ---")
set(ECTO_ROS_VERSION 0.1.0)
set(ECTO_ROS_PREFIX ${PROJECT_NAME}-${ECTO_ROS_VERSION} CACHE PATH "Mangled prefixer" FORCE)
set(ECTO_ROS_INCLUDE_PREFIX include/${ECTO_ROS_PREFIX} CACHE PATH "Where ecto_ros include will live" FORCE)
set(ECTO_ROS_SHARE_PREFIX share/${ECTO_ROS_PREFIX} CACHE PATH "Where ecto_ros share will live" FORCE)

#rosbuild light
find_program(ROSPACK_EXE rospack)
set(BUILD_ECTO_ROS ON)

if (NOT ROSPACK_EXE)
  message("*** unable to find rospack... is your environment set correctly?")
  set (BUILD_ECTO_ROS OFF)
endif()

if ("$ENV{ROS_PACKAGE_PATH}" STREQUAL "")
  message("*** ROS_PACKAGE_PATH is not set... is your environment set correctly?")
  set (BUILD_ECTO_ROS OFF)
endif()  
   
if ("$ENV{ROS_ROOT}" STREQUAL "")
  message("*** ROS_ROOT is not set... is your environment set correctly?")
  set (BUILD_ECTO_ROS OFF)
endif()  
   
if (NOT BUILD_ECTO_ROS)
  message("*** Disabling build of ecto_ros")
  return()
endif()

set(CMAKE_MODULE_PATH ${ecto_ros_SOURCE_DIR}/cmake)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
include(rosbuild_lite)

include_directories(${ecto_ros_SOURCE_DIR}/include)
add_subdirectory(include/ecto_ros)
add_subdirectory(src)
add_subdirectory(cmake)
