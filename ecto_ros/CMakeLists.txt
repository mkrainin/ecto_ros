cmake_minimum_required(VERSION 2.8)
project(ecto_ros)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

include_directories(${CMAKE_SOURCE_DIR}/include)
set(ECTO_ROS_VERSION 0.1.0)
set(ECTO_ROS_PREFIX ${PROJECT_NAME}-${ECTO_ROS_VERSION} CACHE PATH "Mangled prefixer" FORCE)

set(ECTO_ROS_INCLUDE_PREFIX include/${ECTO_ROS_PREFIX} CACHE PATH "Where ecto_ros include will live" FORCE)
set(ECTO_ROS_SHARE_PREFIX share/${ECTO_ROS_PREFIX} CACHE PATH "Where ecto_ros share will live" FORCE)

add_subdirectory(include/ecto_ros)
add_subdirectory(src)
add_subdirectory(cmake)
