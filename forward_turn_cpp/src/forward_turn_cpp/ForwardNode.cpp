// Copyright 2024 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

#include "forward_turn_cpp/ForwardNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace forward_turn_cpp
{

ForwardNode::ForwardNode()
: Node ("forward_node"),
tf_buffer_(),
tf_listener_(tf_buffer_)
{
  linear_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_pos_check_ = create_wall_timer(
    50ms, std::bind(&ForwardNode::transform_callback, this));
  timer_publish_ = create_wall_timer(
    50ms, std::bind(&ForwardNode::linear_move, this));
}

void
ForwardNode::transform_callback()
{
  tf2::Stamped<tf2::Transform> odom2bf;
  std::string error;

  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    double x = odom2bf_msg.transform.translation.x;
    double y = odom2bf_msg.transform.translation.y;
    
    dist = sqrt(x * x + y * y);

    RCLCPP_INFO(get_logger(), "Distance between odom and base_footprint: %f", dist);
  }
}

void
ForwardNode::linear_move()
{
  RCLCPP_INFO(get_logger(), "Distance: %f", dist);
  if (dist < 1.0)
  {
    RCLCPP_INFO(get_logger(), "Moving forward!");
    l_vel_.linear.x = MOVE_SPEED;
    linear_->publish(l_vel_);
  }

  else
  {
    RCLCPP_INFO(get_logger(), "Stopping!");
    l_vel_.linear.x = STOP_SPEED;
    linear_->publish(l_vel_);
  }
  
}

}  //  namespace forward_turn_cpp