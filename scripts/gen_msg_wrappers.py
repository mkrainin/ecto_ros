#!/usr/bin/env python
import subprocess,sys,os
source_code='''/* DO NOT EDIT or check into source control
 * Generated code for wrapping a ros message Pub/Sub in ecto
 *
 * package : %(msg_pkg)s
 * msg : %(msg_type)s
 */
#include <ecto_ros/wrap_sub.hpp>
#include <ecto_ros/wrap_pub.hpp>

#include <%(msg_pkg)s/%(msg_type)s.h>

namespace ecto_%(msg_pkg)s
{
    struct Subscriber_%(msg_type)s : ecto_ros::Subscriber<%(msg_pkg)s::%(msg_type)s> {};
    struct Publisher_%(msg_type)s : ecto_ros::Publisher<%(msg_pkg)s::%(msg_type)s> {};
}

ECTO_CELL(ecto_%(msg_pkg)s, ecto_%(msg_pkg)s::Subscriber_%(msg_type)s,"Subscriber_%(msg_type)s", "Subscribes to a %(msg_pkg)s::%(msg_type)s.");
ECTO_CELL(ecto_%(msg_pkg)s, ecto_%(msg_pkg)s::Publisher_%(msg_type)s,"Publisher_%(msg_type)s", "Publishes a %(msg_pkg)s::%(msg_type)s.");

'''

module_code='''/* DO NOT EDIT or check into source control
 * Generated code for wrapping a ros package with ecto
 * package : %(msg_pkg)s
 */
#include <ecto/ecto.hpp>

ECTO_DEFINE_MODULE(ecto_%(msg_pkg)s) {}

'''

if __name__ == "__main__":
    if len(sys.argv) != 2 or '-h' in sys.argv:
        print '''Usage:
./gen_msg_wrappers.py <ROS_PACKAGE>'''
        sys.exit(1)
    
    msg_pkg = sys.argv[1]
    (o,e) = subprocess.Popen(['rosmsg','package',msg_pkg],stdout=subprocess.PIPE).communicate()
    #strip newlines
    msgs = [x.strip() for x in o.split('\n')]
    filename = 'ecto_%(msg_pkg)s.cpp'%locals()
    print filename
    with open(filename, 'wt') as module_source_code:
        module_source_code.write(module_code%locals())
        
    for msg in msgs:
        if len(msg) == 0 : continue
        msg_pkg = msg.split('/')[0]
        msg_type = msg.split('/')[1]
        filename = 'wrap_%(msg_pkg)s_%(msg_type)s.cpp'%locals()
        print filename
        with open(filename, 'wt') as msg_source_code:
            msg_source_code.write(source_code%locals())