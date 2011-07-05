#!/usr/bin/python
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto
import ecto_ros, ecto_sensor_msgs
from ecto_opencv import highgui
import sys
import argparse

def do_ecto(device_id=0,frame_id='base', threads=1):
    video_capture = highgui.VideoCapture('Video Camera',video_device=device_id)
    mat2image = ecto_ros.Mat2Image(frame_id=frame_id,encoding='bgr8')
    pub_rgb = ecto_sensor_msgs.Publisher_Image("image pub", topic_name='image')
    graph = [
                video_capture["image"] >> mat2image["image"],
                mat2image["image"] >> pub_rgb["input"]
            ]
    
    plasm = ecto.Plasm()
    plasm.connect(graph)
    if threads == 1:
        sched = ecto.schedulers.Singlethreaded(plasm)
        sched.execute()
    else:
        sched = ecto.schedulers.Threadpool(plasm)
        sched.execute(threads)
if __name__ == "__main__":
    ecto_ros.init(sys.argv, "image_pub")
    
    parser = argparse.ArgumentParser(description='Turns a /dev/video* into a ros image publisher.')
    parser.add_argument('--device_id', metavar='N',  dest='device_id', type=int,default=0,
                       help='the device id to open.')
    parser.add_argument('--frame_id', dest='frame_id', type=str, default='base',
                       help='The frame id to associate this camera with.')
    parser.add_argument('--threads', dest='threads', type=int, default=1,
                       help='The number of threads to run with.')
    args = parser.parse_args()
    
    do_ecto(args.device_id,args.frame_id,args.threads)
