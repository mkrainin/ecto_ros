find_package(ecto REQUIRED)
find_package(OpenCV REQUIRED)

find_ros_package(roscpp)
find_ros_package(geometry_msgs)
find_ros_package(sensor_msgs)
find_ros_package(message_filters)

ectomodule(ecto_ros
    ecto_ros.cpp
    camera_sub.cpp
    cv_bridge.cpp
    message_sub.cpp
    message_pub.cpp
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

