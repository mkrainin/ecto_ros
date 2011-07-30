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
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>

namespace ecto_ros
{

  using ecto::tendrils;
  using std::string;
  using namespace sensor_msgs;

  struct CameraInfo2Cv
  {
    static void
    declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<CameraInfoConstPtr>("camera_info");
      o.declare<cv::Mat>("K");
      o.declare<cv::Mat>("D");
      o.declare<cv::Size>("image_size");
    }
    void
    configure(tendrils& p, tendrils& i, tendrils& o)
    {
      camera_info_ = i["camera_info"];
      K_ = o["K"];
      D_ = o["D"];
      image_size_ = o["image_size"];
    }
    int
    process(const tendrils&, tendrils&)
    {
      CameraInfo ci = **camera_info_;
      cv::Mat K(3, 3, CV_64FC1);
      for (int i = 0; i < 9; i++)
        K.at<double>(i / 3, i % 3) = ci.K[i];
      cv::Mat D(ci.D.size(), 1, CV_64FC1);
      for (size_t i = 0; i < ci.D.size(); i++)
      {
        D.at<double>(i, 1) = ci.D[i];
      }
      cv::Size s(ci.width, ci.height);
      *image_size_ = s;
      *K_ = K;
      return ecto::OK;
    }
    ecto::spore<CameraInfoConstPtr> camera_info_;
    ecto::spore<cv::Mat> K_, D_;
    ecto::spore<cv::Size> image_size_;
  };
}
ECTO_CELL(ecto_ros, ecto_ros::CameraInfo2Cv, "CameraInfo2Cv", "Takes a CameraInfo message and converts to OpenCV types.");
