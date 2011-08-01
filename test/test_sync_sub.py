#!/usr/bin/env python
import ecto
import ecto_ros, ecto_sensor_msgs
from ecto_opencv import highgui
import sys
from ecto_ros_test_utils import *

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

def do_ecto(bagname, msg_counts, Scheduler):
    ecto_ros.init(sys.argv, "image_sub_node")
    subs = dict(
                image=ImageSub(topic_name='/camera/rgb/image_color', queue_size=0),
                depth=ImageSub(topic_name='/camera/depth/image', queue_size=0),
                depth_info=CameraInfoSub(topic_name='/camera/depth/camera_info', queue_size=0),
                image_info=CameraInfoSub(topic_name='/camera/rgb/camera_info', queue_size=0),
             )
    
    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs
                                 )
    counter_rgb = ecto.Counter()
    counter_depth = ecto.Counter()
    counter_rgb_info = ecto.Counter()
    counter_depth_info = ecto.Counter()

    graph = [
                sync["image"] >> counter_rgb[:],
                sync["depth"] >> counter_depth[:],
                sync["image_info"] >> counter_rgb_info[:],
                sync["depth_info"] >> counter_depth_info[:],
            ]
    plasm = ecto.Plasm()
    plasm.connect(graph)
    sched = Scheduler(plasm)
    sched.execute_async()
    rosbag = play_bag(bagname, delay=3, rate=1)
    wait_bag(rosbag)
    time.sleep(1)
    sched.stop()

    print "expecting RGB count:", msg_counts['/camera/rgb/image_color']
    print "RGB count:", counter_rgb.outputs.count
    # assert msg_counts['/camera/rgb/image_color'] == counter_rgb.outputs.count
    
if __name__ == "__main__":
    bagname = sys.argv[1]
    msg_counts = bag_counts(bagname)
    try:
      roscore = start_roscore(delay=2)
      do_ecto(bagname, msg_counts, ecto.schedulers.Singlethreaded)
    finally:
      roscore.terminate()
    
