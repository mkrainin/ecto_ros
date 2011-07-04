#!/usr/bin/python
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto
import ecto_ros, ecto_sensor_msgs
import sys

def do_ecto():
    sub_rgb = ecto_sensor_msgs.Subscriber_Image("image_sub", topic_name='camera/rgb/image_mono')
    sub_depth = ecto_sensor_msgs.Subscriber_Image("depth_sub", topic_name='camera/depth/image')
    pub_rgb = ecto_sensor_msgs.Publisher_Image("image_pub", topic_name='my_image')
    pub_depth = ecto_sensor_msgs.Publisher_Image("depth_pub", topic_name='my_depth')
    graph = [
                sub_rgb["output"] >> pub_rgb["input"],
                sub_depth["output"] >> pub_depth["input"]
            ]
    
    plasm = ecto.Plasm()
    plasm.connect(graph)
    ecto.view_plasm(plasm)
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(8)

if __name__ == "__main__":
    ecto_ros.init(sys.argv, "image_pub")
    do_ecto()
