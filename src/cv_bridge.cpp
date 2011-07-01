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
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <string>

namespace
{
  namespace enc = sensor_msgs::image_encodings;
  int getCvType(const std::string& encoding)
  {
    // Check for the most common encodings first
    if (encoding == enc::BGR8)
      return CV_8UC3;
    if (encoding == enc::MONO8)
      return CV_8UC1;
    if (encoding == enc::RGB8)
      return CV_8UC3;
    if (encoding == enc::MONO16)
      return CV_16UC1;
    if (encoding == enc::BGRA8)
      return CV_8UC4;
    if (encoding == enc::RGBA8)
      return CV_8UC4;

    // For bayer, return one-channel
    if (encoding == enc::BAYER_RGGB8)
      return CV_8UC1;
    if (encoding == enc::BAYER_BGGR8)
      return CV_8UC1;
    if (encoding == enc::BAYER_GBRG8)
      return CV_8UC1;
    if (encoding == enc::BAYER_GRBG8)
      return CV_8UC1;

    // Check all the generic content encodings
#define CHECK_ENCODING(code)                            \
    if (encoding == enc::TYPE_##code) return CV_##code    \
    /***/
#define CHECK_CHANNEL_TYPE(t)                   \
    CHECK_ENCODING(t##1);                         \
    CHECK_ENCODING(t##2);                         \
    CHECK_ENCODING(t##3);                         \
    CHECK_ENCODING(t##4);                         \
    /***/

    CHECK_CHANNEL_TYPE(8UC);
    CHECK_CHANNEL_TYPE(8SC);
    CHECK_CHANNEL_TYPE(16UC);
    CHECK_CHANNEL_TYPE(16SC);
    CHECK_CHANNEL_TYPE(32SC);
    CHECK_CHANNEL_TYPE(32FC);
    CHECK_CHANNEL_TYPE(64FC);

#undef CHECK_CHANNEL_TYPE
#undef CHECK_ENCODING

    throw std::runtime_error("Unrecognized image encoding [" + encoding + "]");
  }
}
namespace ros
{

  using ecto::tendrils;
  using std::string;
  using namespace sensor_msgs;

  struct Image2Mat
  {
    static void declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<ImageConstPtr> ("image",
                                "A sensor_msg::Image message from ros.");

      o.declare<cv::Mat> ("image", "A cv::Mat copy.");
    }
    int process(const tendrils& i, tendrils& o)
    {
      ImageConstPtr image = i.get<ImageConstPtr> ("image");
      cv::Mat& mat = o.get<cv::Mat> ("image");

      // Construct matrix pointing to source data
      int source_type = getCvType(image->encoding);
      cv::Mat
          temp((int) image->height, (int) image->width, source_type,
               const_cast<uint8_t*> (&image->data[0]), (size_t) image->step);
      temp.copyTo(mat);
      return ecto::OK;
    }
  };

}

ECTO_MODULE(ecto_ros, ros::Image2Mat, "Image2Mat", "Converts an Image message to cv::Mat type.");

