#include <ecto/ecto.hpp>
#include <boost/python/stl_iterator.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <string>

namespace bp = boost::python;
void ros_init_wtf(bp::object sys_argv, const std::string& node_name)
{
  std::vector<std::string> args;
  bp::stl_input_iterator<std::string> begin(sys_argv), end;
  std::copy(begin, end, std::back_inserter(args));
  char ** argv = new char*[args.size()];
  for (int i = 0, ie = args.size(); i < ie; ++i)
  {
    argv[i] = const_cast<char*>(args[i].data());
  }
  int ac = args.size();
  ros::init(ac, argv, node_name.c_str(), ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  delete [] argv;
}

ECTO_DEFINE_MODULE(ecto_ros)
{
  bp::def("init",ros_init_wtf, "Calls roscpp initialization routine. Please call with sys.argv, or similar list of commandline args. Will not strip them...");
}
