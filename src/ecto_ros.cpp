#include <ecto/ecto.hpp>
#include <ros/ros.h>

#include <boost/python/stl_iterator.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/python/overloads.hpp>

#include <iostream>
#include <string>

namespace
{
  struct Spinner
  {
    void
    operator()()
    {
      ros::spin();
    }
  };

  struct RosLifetime
  {
    RosLifetime()
        :
          thread_(new boost::thread(Spinner()))
    {
    }
    ~RosLifetime()
    {
      if (ros::ok())
      {
        ros::requestShutdown();
      }
      thread_->join();
    }
    boost::scoped_ptr<boost::thread> thread_;
    static boost::scoped_ptr<RosLifetime> its_a_ros_life_;
  };

  boost::scoped_ptr<RosLifetime> RosLifetime::its_a_ros_life_;
  namespace bp = boost::python;


  void
  ros_init(bp::list sys_argv, const std::string& node_name, bool anonymous = true)
  {
    std::vector<std::string> args;
    bp::stl_input_iterator<std::string> begin(sys_argv), end;
    std::copy(begin, end, std::back_inserter(args));
    char** argv = new char*[args.size()]; //array to emulate argv.
    for (int i = 0, ie = args.size(); i < ie; ++i)
    {
      argv[i] = const_cast<char*>(args[i].data());
    }

    int ac = args.size();
    int flags = ros::init_options::NoSigintHandler;
    if(anonymous)
    {
      flags |= ros::init_options::AnonymousName;
    }
    ros::init(ac, argv, node_name.c_str(), flags);

    //forward the arg stripping back to python
    while (bp::len(sys_argv))
      sys_argv.pop();

    for (int i = 0; i < ac; ++i)
    {
      sys_argv.append(bp::str((const char*) (argv[i])));
    }
    delete[] argv;
    RosLifetime::its_a_ros_life_.reset(new RosLifetime());
  }
  BOOST_PYTHON_FUNCTION_OVERLOADS(ros_init_overloads, ros_init, 2, 3)
}

ECTO_DEFINE_MODULE(ecto_ros)
{
  bp::def(
      "init",
      ros_init,
      ros_init_overloads()
      );
 }
