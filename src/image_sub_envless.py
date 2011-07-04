#!/usr/bin/python
#PKG = 'ecto_ros' # this package name
#import roslib; roslib.load_manifest(PKG)
import ecto
import ecto_ros, ecto_sensor_msgs
from ecto_opencv import highgui
import sys

def do_ecto():
    sub_rgb = ecto_sensor_msgs.Subscriber_Image("image_sub",topic_name='camera_rgb')
    sub_depth = ecto_sensor_msgs.Subscriber_Image("depth_sub",topic_name='camera_depth')

    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()

    s1 = ecto.Strand() #imshow is not thread safe.
    
    graph = [
                sub_rgb["output"]>>im2mat_rgb["image"],
                im2mat_rgb["image"] >> highgui.imshow("rgb show",name="rgb", waitKey=5, strand=s1)[:],
                sub_depth["output"]>> im2mat_depth["image"],
                im2mat_depth["image"] >> highgui.imshow("depth show",name="depth", waitKey=-1, strand=s1)[:]
            ]
    plasm = ecto.Plasm()
    plasm.connect(graph)
    ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(8)

if __name__ == "__main__":
    ecto_ros.init(sys.argv,"image_node")
    do_ecto()
