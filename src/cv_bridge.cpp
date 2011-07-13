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

  std::string mattype2enconding(int mat_type)
  {
    switch(mat_type)
    {
      case CV_8UC1:
        return enc::MONO8;
      case CV_8UC3:
        return enc::RGB8;
      case CV_16SC1:
        return enc::MONO16;
      case CV_8UC4:
        return enc::RGBA8;
      default:
        break;
    }

#define CASE_ENCODING(code)                    \
    case CV_##code : return enc::TYPE_##code; do{}while(false)    \
    /***/
#define CASE_CHANNEL_TYPE(t)                   \
    CASE_ENCODING(t##1);                         \
    CASE_ENCODING(t##2);                         \
    CASE_ENCODING(t##3);                         \
    CASE_ENCODING(t##4);                         \
    /***/
    switch(mat_type)
    {
      CASE_CHANNEL_TYPE(8UC);
      CASE_CHANNEL_TYPE(8SC);
      CASE_CHANNEL_TYPE(16UC);
      CASE_CHANNEL_TYPE(16SC);
      CASE_CHANNEL_TYPE(32SC);
      CASE_CHANNEL_TYPE(32FC);
      CASE_CHANNEL_TYPE(64FC);
      default:
        throw std::runtime_error("Unknown encoding type.");
    }

  }
  void toImageMsg(const cv::Mat& mat, sensor_msgs::Image& ros_image)
  {
    ros_image.height = mat.rows;
    ros_image.width = mat.cols;
    ros_image.encoding = mattype2enconding(mat.type());
    ros_image.is_bigendian = false;
    ros_image.step = mat.step;
    size_t size = mat.step * mat.rows;
    ros_image.data.resize(size);
    memcpy((char*)(&ros_image.data[0]), mat.data, size);
  }

}
namespace ecto_ros
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
    void configure(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      image_msg_ = i.at("image");
      mat_ = o.at("image");
    }
    int process(const tendrils& i, tendrils& o)
    {
      ImageConstPtr image = *image_msg_;
      cv::Mat& mat = *mat_;
      // Construct matrix pointing to source data
      int source_type = getCvType(image->encoding);
      cv::Mat
          temp((int) image->height, (int) image->width, source_type,
               const_cast<uint8_t*> (&image->data[0]), (size_t) image->step);
      temp.copyTo(mat);
      return ecto::OK;
    }
    ecto::spore<ImageConstPtr> image_msg_;
    ecto::spore<cv::Mat> mat_;

  };

  struct Mat2Image
  {
    static void
    declare_params(tendrils& p)
    {
      p.declare<std::string>("frame_id","Frame this data is associated with","default_frame");
      p.declare<std::string>("encoding","ROS image message encoding override.");


    }

    static void declare_io(const tendrils& /*p*/, tendrils& i, tendrils& o)
    {
      i.declare<cv::Mat> ("image", "A cv::Mat.");
      o.declare<ImageConstPtr> ("image",
                                "A sensor_msg::Image message.");
    }

    void configure(tendrils& p, tendrils& i, tendrils& o)
    {
      mat_ = i.at("image");
      image_msg_out_ = o.at("image");
      frame_id_ = p.get<std::string>("frame_id");
      header_.frame_id = frame_id_;
      encoding_ = p.at("encoding");
    }
    int process(const tendrils& i, tendrils& o)
    {
      ImagePtr image_msg(new Image);
      toImageMsg(*mat_,*image_msg);
      if(encoding_.user_supplied())
      {
        image_msg->encoding = encoding_.read();
      }
      header_.seq++;
      header_.stamp = ros::Time::now();
      image_msg->header = header_;
      *image_msg_out_ =image_msg;
      return ecto::OK;
    }
    std_msgs::Header header_;
    std::string frame_id_;
    ecto::spore<ImageConstPtr> image_msg_out_;
    ecto::spore<cv::Mat> mat_;
    ecto::spore<std::string> encoding_;

  };

}

ECTO_CELL(ecto_ros, ecto_ros::Image2Mat, "Image2Mat", "Converts an Image message to cv::Mat type.");
ECTO_CELL(ecto_ros, ecto_ros::Mat2Image, "Mat2Image", "Converts an cv::Mat to Image message type.");

