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
#pragma once
#include <ecto/ecto.hpp>
#include <ros/ros.h>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace ecto_ros
{
  /**
   * \brief Use this to wrap a simple ros message subscriber.
   */
  template<typename MessageT>
  struct Subscriber
  {
    typedef typename MessageT::ConstPtr MessageConstPtr;
    //ros subscription stuffs
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string topic_;
    int buffer_;
    boost::condition_variable cond_;
    boost::mutex mut_;
    MessageConstPtr data_;
    ecto::spore<MessageConstPtr> out_;

    Subscriber()
    {
    }

    void
    setupSubs()
    {
      //look up remapping
      std::string topic = nh_.resolveName(topic_, true);
      sub_ = nh_.subscribe(topic, buffer_, &Subscriber::dataCallback, this);
      ROS_INFO_STREAM("subscribed to topic:" << topic);
    }

    void
    dataCallback(const typename MessageT::ConstPtr& data)
    {
      {
        boost::lock_guard<boost::mutex> lock(mut_);
        data_ = data;
      }
      cond_.notify_one();
    }

    static void
    declare_params(ecto::tendrils& params)
    {
      params.declare<std::string> ("topic_name", "The topic name to subscribe to.", "topic_foo");
      params.declare<int> ("buffer_size", "The amount to buffer incoming messages.", 2);
    }

    static void
    declare_io(const ecto::tendrils& p, ecto::tendrils& in, ecto::tendrils& out)
    {
      out.declare<MessageConstPtr> ("output", "The received message.");
    }

    void
    configure(ecto::tendrils& p, ecto::tendrils& in, ecto::tendrils& out)
    {
      topic_ = p.get<std::string> ("topic_name");
      buffer_ = p.get<int> ("buffer_size");
      out_ = out.at("output");
      setupSubs();
    }

    int
    process(const ecto::tendrils& in, ecto::tendrils& out)
    {
      {
        boost::unique_lock<boost::mutex> lock(mut_);
        while (!data_)
        {
          cond_.wait(lock);
        }
        *out_ = data_;
        data_.reset();
      }
      return ecto::OK;
    }
  };
}
