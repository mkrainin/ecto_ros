macro (rospack VAR)
  if(NOT ${VAR}_CACHED)
    execute_process(COMMAND /usr/bin/env rospack ${ARGN}
      OUTPUT_VARIABLE ${VAR}
      ERROR_VARIABLE rospack_error
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
    if(rospack_error)
      message(FATAL_ERROR "Is your path setup correctly for ROS?\nrospack failed:${rospack_error}")
    endif()
    separate_arguments(${VAR} UNIX_COMMAND ${${VAR}})
    set(${VAR} ${${VAR}} CACHE STRING "${VAR} value")
    set(${VAR}_CACHED TRUE CACHE BOOL "${VAR} cached flag")
    message(STATUS "${VAR} := ${${VAR}}")
  else()
    #message(STATUS "BANG! Using cached ${VAR}")
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
