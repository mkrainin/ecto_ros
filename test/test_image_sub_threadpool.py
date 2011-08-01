#!/usr/bin/env python
import ecto
import ecto_ros, ecto_sensor_msgs
from ecto_opencv import highgui
import sys
import subprocess
import time
from ecto_ros_test_utils import *
from test_image_sub import *

if __name__ == "__main__":
    bagname = sys.argv[1]
    msg_counts = bag_counts(bagname)
    try:
      roscore = start_roscore(delay=1)
      ecto_ros.init(sys.argv, "image_sub_node")
      do_ecto(bagname, msg_counts, ecto.schedulers.Threadpool)
    finally:
      roscore.terminate()
    
