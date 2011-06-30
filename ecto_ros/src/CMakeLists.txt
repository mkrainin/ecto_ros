find_package(ecto REQUIRED)
find_package(OpenCV REQUIRED)

ectorosmodule(ecto_ros
    ecto_ros.cpp
    camera_sub.cpp
    cv_bridge.cpp
    message_sub.cpp
)

link_ecto(ecto_ros
    ${OpenCV_LIBS}
)
