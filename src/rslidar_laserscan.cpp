/*******************************************************************************************
 *
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install, copy or use the software.

License Agreement
For Rslidar Laserscan Tool
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list
of conditions and the following disclaimer in the documentation and/or other materials
provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names
of other contributors may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

********************************************************************************************/
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>

#include "rslidar_laserscan/rslidar_laserscan.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "std_msgs/msg/string.hpp"
#include <functional>
#include <memory>
using std::placeholders::_1;

namespace rslidar_laserscan
{
RslidarLaserScan::RslidarLaserScan(std::shared_ptr<rclcpp::Node> nh, std::shared_ptr<rclcpp::Node> nh_priv) : nh_(nh)
{
  // ros::SubscriberStatusCallback connect_cb = boost::bind(&RslidarLaserScan::connectCb, this); 

  // nh_priv.param("sub_topic", sub_topic_, std::string("/rslidar_points"));
  rcl_interfaces::msg::ParameterDescriptor rslidar_points;
  rslidar_points.name = "/rslidar_points";
  rslidar_points.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  auto rs_lidar_points_ = nh_->declare_parameter("sub_topic", sub_topic_, rslidar_points);

  std::string model = "RS16";
  rcl_interfaces::msg::ParameterDescriptor RS16;
  RS16.name = "RS16";
  RS16.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  // nh_priv.param("model", model, std::string("RS16"));
  auto model_ = nh_->declare_parameter("model", model, RS16);

  int ring = 1000;
  rcl_interfaces::msg::ParameterDescriptor ring_1000;
  ring_1000.name = "ring";
  ring_1000.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  // nh_priv.param("ring", ring, 1000);  // 1000 is not set ring
  nh_->declare_parameter("ring", ring, ring_1000);

  if (model == "RS16")
  {
    height_ = 16;
    if (ring != 1000 && (ring > 0 && ring < 16 + 1))
    {
      ring_ = ring - 1;  // the code start from 0
      std::cout << "set ring: " << ring << std::endl;
    }
    else
    {
      ring_ = 7;
    }
    range_min_ = 0.4;
    range_max_ = 150.0;
  }
  else if (model == "RS32")
  {
    height_ = 32;
    if (ring != 1000 && (ring > 0 && ring < 32 + 1))
    {
      ring_ = ring - 1;  // the code start from 0
      std::cout << "set ring: " << ring << std::endl;
    }
    else
    {
      ring_ = 9;
    }
    range_min_ = 0.4;
    range_max_ = 200.0;
  }
  else if (model == "RSHELIOS")
  {
    height_ = 32;
    if (ring != 1000 && (ring > 0 && ring < 32 + 1))
    {
      ring_ = ring - 1;  // the code start from 0
      std::cout << "set ring: " << ring << std::endl;
    }
    else
    {
      ring_ = 9;
    }
    range_min_ = 0.2;
    range_max_ = 150.0;
  }
  else if (model == "RSHELIOS_16P")
  {
    height_ = 16;
    if (ring != 1000 && (ring > 0 && ring < 16 + 1))
    {
      ring_ = ring - 1;  // the code start from 0
      std::cout << "set ring: " << ring << std::endl;
    }
    else
    {
      ring_ = 6;
    }
    range_min_ = 0.4;
    range_max_ = 200.0;
  }
  else if (model == "RSBP")
  {
    height_ = 32;
    if (ring != 1000 && (ring > 0 && ring < 32 + 1))
    {
      ring_ = ring - 1;  // the code start from 0
      std::cout << "set ring: " << ring << std::endl;
    }
    else
    {
      ring_ = 31;
    }
    range_min_ = 0.1;
    range_max_ = 30.0;
  }
  else if (model == "RS80")
  {
    height_ = 80;
    if (ring != 1000 && (ring > 0 && ring < 80 + 1))
    {
      ring_ = ring - 1;  // the code start from 0
      std::cout << "set ring: " << ring << std::endl;
    }
    else
    {
      ring_ = 74;
    }
    range_min_ = 1.0;
    range_max_ = 230.0;
  }
  else if (model == "RS128")
  {
    height_ = 128;
    if (ring != 1000 && (ring > 0 && ring < 128 + 1))
    {
      ring_ = ring - 1;  // the code start from 0
      std::cout << "set ring: " << ring << std::endl;
    }
    else
    {
      ring_ = 118;
    }
    range_min_ = 1.0;
    range_max_ = 250.0;
  }
  else
  {
    std::cout << "lidar model is bad. please choose right model from: RS16|RS32|RSHELIOS|RSHELIOS_16P|RSBP|RS80|RS128!" << std::endl;
    exit(-1);
  }
  pub_ = nh_->create_publisher<sensor_msgs::msg::LaserScan>("rslidar_laserscan", 10
  // , std::bind(&RslidarLaserScan::connectCb, this)
  );
}

void RslidarLaserScan::connectCb()
{
  boost::lock_guard<std::mutex> lock(connect_mutex_);
  if (!pub_->get_subscription_count())
  {
    // sub_.shutdown();
    rclcpp::shutdown();
  }
  else if (!sub_)
  {
    sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
                  "rslidar_points", rclcpp::QoS(10), //&RslidarLaserScan::recvCallback, this
                  std::bind(&RslidarLaserScan::recvCallback, nh_, std::placeholders::_1)
    );
  }
}

void RslidarLaserScan::recvCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // RCLCPP_INFO_ONCE("RobosenseLaserScan. point cloud width %u, height %u, extracting ring %u", msg->width, msg->height,
  //               ring_);

  // field offsets
  int offset_x = -1;
  int offset_y = -1;
  int offset_z = -1;
  int offset_i = -1;

  for (size_t i = 0; i < msg->fields.size(); i++)
  {
    if (msg->fields[i].datatype == sensor_msgs::msg::PointField::FLOAT32)
    {
      if (msg->fields[i].name == "x")
      {
        offset_x = msg->fields[i].offset;
      }
      else if (msg->fields[i].name == "y")
      {
        offset_y = msg->fields[i].offset;
      }
      else if (msg->fields[i].name == "z")
      {
        offset_z = msg->fields[i].offset;
      }
      else if (msg->fields[i].name == "intensity")
      {
        offset_i = msg->fields[i].offset;
      }
    }
  }

  // ROS_INFO_ONCE ("offset x:%u, y:%u, z:%u, intensity:%u",
  // offset_x, offset_y, offset_z, offset_i);

  // laser scan msg
  if ((offset_x == 0) && (offset_y >= 0) && (offset_z >= 0) && (offset_i >= 0))
  {
    const float RESOLUTION = 0.0034906584;  // 2.0 * M_PI / 1800 (10Hz)
    const size_t SIZE = 2.0 * M_PI / RESOLUTION;

    // std::shared_ptr<sensor_msgs::msg::LaserScan> scan(new sensor_msgs::msg::LaserScan());
    auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan->header = msg->header;
    scan->angle_increment = RESOLUTION;

    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;

    scan->range_min = range_min_;
    scan->range_max = range_max_;

    scan->time_increment = 0.0;
    scan->ranges.resize(SIZE, INFINITY);
    scan->intensities.resize(SIZE);

    size_t i = 0;
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++i, ++it)
    {
      if ((i % height_) == ring_)
      {
        static const size_t X = 0;
        static const size_t Y = offset_y / 4;
        static const size_t I = offset_i / 4;

        const float x = it[X];          // x
        const float y = it[Y];          // y
        const float intensity = it[I];  // intensity

        const int bin = (atan2f(y, x) + static_cast<float>(M_PI)) / RESOLUTION;
        if ((bin >= 0) && (bin < static_cast<int>(SIZE)))
        {
          scan->ranges[bin] = sqrtf(x * x + y * y);
          scan->intensities[bin] = intensity;
        }
      }
    }
    pub_->publish(std::move(scan));
  }
}
}
