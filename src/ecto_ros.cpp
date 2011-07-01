#include <ecto/ecto.hpp>
#include <ros/ros.h>

#include <boost/python/stl_iterator.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <string>

struct Spinner
{
  void
  operator()()
  {
    ros::spin();
  }
  static boost::scoped_ptr<boost::thread> thread_;
};

boost::scoped_ptr<boost::thread> Spinner::thread_;

namespace bp = boost::python;
void
ros_init_wtf(bp::object sys_argv, const std::string& node_name)
{
  std::vector<std::string> args;
  bp::stl_input_iterator<std::string> begin(sys_argv), end;
  std::copy(begin, end, std::back_inserter(args));
  char ** argv = new char*[args.size()];
  for (int i = 0, ie = args.size(); i < ie; ++i)
  {
    argv[i] = const_cast<char*> (args[i].data());
  }
  int ac = args.size();
  ros::init(ac, argv, node_name.c_str(), ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  delete[] argv;

  Spinner::thread_.reset(new boost::thread(Spinner()));
}

ECTO_DEFINE_MODULE(ecto_ros)
{
  bp::def("init",ros_init_wtf, "Calls roscpp initialization routine. Please call with sys.argv, or similar list of commandline args. Will not strip them...");
}
