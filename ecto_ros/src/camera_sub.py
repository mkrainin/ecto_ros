#!/usr/bin/python
import ecto
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto_ros
import ecto_opencv
import sys
def do_ecto():
    camera = ecto_ros.ImageDepthSub()
    imshow = ecto_opencv.imshow("rgb", waitKey=10)
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    
    plasm = ecto.Plasm()
    
    plasm.connect(camera["image"]>>im2mat_rgb["image"],
                  camera["depth"]>>im2mat_depth["image"],
                  im2mat_depth["image"] >> imshow["input"],
                  )
    ecto.view_plasm(plasm)
    
    while True:
      plasm.execute()

if __name__ == "__main__":
    ecto_ros.init(sys.argv,"ecto_node")
    do_ecto()