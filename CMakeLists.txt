cmake_minimum_required(VERSION 2.8)
project(ecto_ros)
set(ECTO_ROS_VERSION 0.1.0)
set(ECTO_ROS_PREFIX ${PROJECT_NAME}-${ECTO_ROS_VERSION} CACHE PATH "Mangled prefixer" FORCE)
set(ECTO_ROS_INCLUDE_PREFIX include/${ECTO_ROS_PREFIX} CACHE PATH "Where ecto_ros include will live" FORCE)
set(ECTO_ROS_SHARE_PREFIX share/${ECTO_ROS_PREFIX} CACHE PATH "Where ecto_ros share will live" FORCE)

if(${CMAKE_SOURCE_DIR} EQUAL ${CMAKE_BINARY_DIR})
MESSAGE(FATAL "You must hate life, as you are doing a cmake build inside of the source dir.")
endif()
#rosbuild light
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(rosbuild_lite)

include_directories(${CMAKE_SOURCE_DIR}/include)
add_subdirectory(include/ecto_ros)
add_subdirectory(src)
add_subdirectory(cmake)
