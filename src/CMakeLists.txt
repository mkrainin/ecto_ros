find_package(ecto REQUIRED)
find_package(OpenCV 2.3.1 EXACT REQUIRED)

find_ros_package(roscpp)
find_ros_package(eigen)
find_ros_package(geometry_msgs)
find_ros_package(sensor_msgs)
find_ros_package(message_filters)

macro(pubsub_gen_wrap ROS_PACKAGE)
    if(NOT ${ROS_PACKAGE}_srcs)
        execute_process(COMMAND ${CMAKE_SOURCE_DIR}/scripts/gen_msg_wrappers.py ${ROS_PACKAGE}
          OUTPUT_VARIABLE ${ROS_PACKAGE}_srcs
          ERROR_VARIABLE ${ROS_PACKAGE}_srcs
          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
          OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        message(STATUS "generated ${ROS_PACKAGE} srcs : ${${ROS_PACKAGE}_srcs}")
        separate_arguments(${ROS_PACKAGE}_srcs UNIX_COMMAND ${${ROS_PACKAGE}_srcs})
        set(_SRCS)
        foreach(_SRC ${${ROS_PACKAGE}_srcs})
            list(APPEND _SRCS ${CMAKE_CURRENT_BINARY_DIR}/${_SRC})
        endforeach()
        set(${ROS_PACKAGE}_srcs ${_SRCS} CACHE STRING "The generated srcs for ${ROS_PACKAGE}")
        
    endif()
    find_ros_package(${ROS_PACKAGE})
    ectomodule(ecto_${ROS_PACKAGE}
        ${${ROS_PACKAGE}_srcs}
    )
    link_ecto(ecto_${ROS_PACKAGE}
        ${roscpp_LIBRARIES}
        ${${ROS_PACKAGE}_LIBRARIES}
    )
    install_ecto_module(ecto_${ROS_PACKAGE})
    set_target_properties(ecto_${ROS_PACKAGE}_ectomodule
      PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE
    )
endmacro()

pubsub_gen_wrap(actionlib_msgs)
pubsub_gen_wrap(diagnostic_msgs)
pubsub_gen_wrap(geometry_msgs)
pubsub_gen_wrap(nav_msgs)
pubsub_gen_wrap(sensor_msgs)


ectomodule(ecto_ros
    ecto_ros.cpp
    camera_sub.cpp
    cv_bridge.cpp
    cv_pose.cpp
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

