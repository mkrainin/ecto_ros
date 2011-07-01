find_package(ecto REQUIRED)
find_package(OpenCV REQUIRED)

macro (rospack VAR)
  if(NOT ${VAR}_CACHED)
    execute_process(COMMAND rospack ${ARGN}
      OUTPUT_VARIABLE ${VAR}
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
    separate_arguments(${VAR} UNIX_COMMAND ${${VAR}})
    set(${VAR} ${${VAR}} CACHE STRING "WoNder Bread")
    set(${VAR}_CACHED TRUE CACHE BOOL "WoNder Bread 2")
    message(STATUS "${VAR} := ${${VAR}}")
  else()
    message(STATUS "Cache bang! ${VAR}")
  endif()
endmacro()

macro (find_ros_package NAME)
  rospack(${NAME}_INCLUDES cflags-only-I ${NAME})
  include_directories(${${NAME}_INCLUDES})
  rospack(${NAME}_DEFINITIONS cflags-only-other ${NAME})
  add_definitions(" ${${NAME}_DEFINITIONS}")
  rospack(${NAME}_LIBRARY_DIRS libs-only-L ${NAME})
  rospack(${NAME}_LIBRARIES libs-only-l ${NAME})
  link_directories(${${NAME}_LIBRARY_DIRS})
endmacro()



find_ros_package(roscpp)
find_ros_package(geometry_msgs)
find_ros_package(sensor_msgs)
find_ros_package(message_filters)

ectomodule(ecto_ros
    ecto_ros.cpp
    camera_sub.cpp
    cv_bridge.cpp
    message_sub.cpp
)

link_ecto(ecto_ros
    ${OpenCV_LIBS}
    ${roscpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${message_filters_LIBRARIES}
)

install_ecto_module(ecto_ros)

set_target_properties(ecto_ros_ectomodule
  PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE
)

