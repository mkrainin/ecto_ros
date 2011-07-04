#!/usr/bin/python
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto
import ecto_ros, ecto_sensor_msgs
from ecto_opencv import highgui
import sys

def do_ecto(device_id=0,frame_id='base'):
    video_capture = highgui.VideoCapture('Video Camera',video_device=device_id)
    mat2image = ecto_ros.Mat2Image(frame_id=frame_id,encoding='bgr8')
    pub_rgb = ecto_sensor_msgs.Publisher_Image("image pub", topic_name='cvcamera/image')
    graph = [
                video_capture["image"] >> mat2image["image"],
                mat2image["image"] >> pub_rgb["input"]
            ]
    
    plasm = ecto.Plasm()
    plasm.connect(graph)
    ecto.view_plasm(plasm)
    sched = ecto.schedulers.Threadpool(plasm)
    try:
        sched.execute(8)
    except RuntimeError, e:
        print e
if __name__ == "__main__":
    ecto_ros.init(sys.argv, "image_pub")
    do_ecto()
