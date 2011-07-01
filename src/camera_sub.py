#!/usr/bin/python
PKG = 'ecto_ros' # this package name
import roslib; roslib.load_manifest(PKG)
import ecto
import ecto_ros
from ecto_opencv import highgui
import sys
def do_ecto():
    camera = ecto_ros.ImageDepthSub()
    print camera.__doc__
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()
    s1 = ecto.Strand() #imshow is not thread safe.
    graph = [
                camera["image"]>>im2mat_rgb["image"],
                camera["depth"]>>im2mat_depth["image"],
                im2mat_depth["image"] >> highgui.imshow("depth show",name="depth", waitKey=10,strand=s1)[:],
                im2mat_rgb["image"] >> highgui.imshow("rgb show",name="rgb", waitKey=-1, strand=s1)[:]
            ]
    plasm = ecto.Plasm()
    plasm.connect(graph)
    ecto.view_plasm(plasm)
    
    sched = ecto.schedulers.Threadpool(plasm)
    sched.execute(8)

if __name__ == "__main__":
    ecto_ros.init(sys.argv,"ecto_node")
    do_ecto()
