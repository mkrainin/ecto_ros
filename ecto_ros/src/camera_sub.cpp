/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ecto/ecto.hpp>

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

#include <opencv2/core/core.hpp>

namespace ros
{

using ecto::tendrils;

struct ImageDepthSub
{
  typedef message_filters::Subscriber<CameraInfo> CameraInfoSubscriber;
  typedef message_filters::Subscriber<Image> ImageSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo> ApproxSyncPolicy;
  typedef message_filters::Synchronizer<ApproxSyncPolicy> SynchronizerImageDepthCamera;

  ImageSubscriber image_sub_, depth_sub_;
  CameraInfoSubscriber camera_info_sub_, depth_camera_info_sub_;
  SynchronizerImageDepthCamera sync_sub_;
  ros::NodeHandle nh_;
  string camera_topic_, depth_camera_topic_;
  ros::Time prev_;
public:
  ImageDepthSub() :
    sync_sub_(10)
  {
  }

  void setupPubs()
  {

  }
  void setupSubs()
  {
    //may be remapped to : /camera/rgb
    camera_topic_ = nh_.resolveName("camera", true);
    depth_camera_topic_ = nh_.resolveName("depth_camera", true);

    ROS_INFO_STREAM("camera topic is set to " << camera_topic_);
    ROS_INFO_STREAM("depth camera topic is set to " <<depth_camera_topic_ );

    //subscribe to rgb camera topics
    image_sub_.subscribe(nh_, camera_topic_ + "/image_color", 2);
    camera_info_sub_.subscribe(nh_, camera_topic_ + "/camera_info", 2);
    
    //subscribe to depth image topics
    depth_sub_.subscribe(nh_, depth_camera_topic_ + "/image", 2);
    depth_camera_info_sub_.subscribe(nh_,  depth_camera_topic_ + "/camera_info", 2);

    //setup the approxiamate time sync
    sync_sub_.connectInput(image_sub_, camera_info_sub_, depth_sub_,depth_camera_info_sub_);
    sync_sub_.registerCallback(&RecognitionNode::dataCallback, this);
  }
  void onInit()
  {
    prev_ = ros::Time::now();

    setupSubs();
    setupPubs();

    ROS_INFO("init done");
  }

  void dataCallback(const ImageConstPtr& image, const CameraInfoConstPtr& camera_info,
                    const ImageConstPtr& depth, const CameraInfoConstPtr& depth_camera_info)
  {
    ros::Time n = image->header.stamp;
    float dt = (n - prev_).toSec();
    ROS_INFO_STREAM("Processing frame bundle. dt=" << dt);
    prev_ = n;
  }
  static void declare_params(tendrils& params)
  {
    params.declare<std::string> ("camera", "RGB camera topic.", "/camera/rgb");
    params.declare<std::string> ("depth_camera", "The depth camera topic.", "/camera/depth");
  }

  static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
  {
    out.declare<cv::Mat>("image","The rgb image");
    out.declare<cv::Mat>("depth","The 16bit single channel depth image, in mm.");
    out.declare<cv::Mat>("K_image","The 3x3 camera intrinsics matrix for the rgb image");
  }

  void configure(tendrils& p)
  {
      std::string camera_remap = boost::format("camera:=%s") % p.get<std::string>("camera");
      std::string depth_camera_remap = boost::format("depth_camera:=%s") % p.get<std::string>("depth_camera");
      int argc = 3;
      char ** argv = ["./camera_sub",camera_remap.c_str(),depth_camera_remap.c_str(),0];
      ros::init(argc, argv , "camera_sub");
      onInit();
  }
  
  int process(const tendrils& in, tendrils& /*out*/)
  {
    std::cout << in.get<std::string> ("str") << std::endl;
    return ecto::OK;
  }
};

struct Reader
{
  static void declare_io(const tendrils& parms, tendrils& in, tendrils& out)
  {
    out.declare<std::string> ("output", "Output from standard in");
  }

  ecto::ReturnCode process(const tendrils& in, tendrils& out)
  {
    std::string o;
    std::cin >> o;
    out.get<std::string> ("output") = o;
    return ecto::eOK;
  }
};

}

BOOST_PYTHON_MODULE(hello_ecto)
{
  using namespace hello_ecto;
  ecto::wrap<Printer>("Printer", "Prints a string input to standard output.");
  ecto::wrap<Reader>("Reader", "Reads input from standard input.");
}
