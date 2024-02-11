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

ForwardTurnNode::ForwardTurnNode()
: Node("forward_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  linear_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  angular_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  timer_pos_check_ = create_wall_timer(
    50ms, std::bind(&ForwardTurnNode::transform_callback, this));
  timer_publish_ = create_wall_timer(
    50ms, std::bind(&ForwardTurnNode::linear_move, this));
}

void
ForwardTurnNode::transform_callback()
{
  tf2::Stamped<tf2::Transform> odom2bf;
  std::string error;

  //  Gets the tf between 'odom' and 'base_footprint'
  if (tf_buffer_.canTransform("odom", "base_footprint", tf2::TimePointZero, &error)) {
    auto odom2bf_msg = tf_buffer_.lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);

    //  Extracts the x and y coordinates from the obtained transformation.
    double x = odom2bf_msg.transform.translation.x;
    double y = odom2bf_msg.transform.translation.y;

    //  Calculate the distance between (0,0) and (x,y)
    distance_ = sqrt(x * x + y * y);
    //  Calculate the distance between (0,0) and (x,y)
    angle_ = atan2(y, x);

    RCLCPP_INFO(get_logger(), "Distance between odom and base_footprint: %f", distance_);
    RCLCPP_INFO(get_logger(), "Angle between odom and base_footprint: %f", angle_);
    RCLCPP_INFO(get_logger(), "Angle between odom and base_footprint: %f", turn_limit_);
  }
}

void
ForwardTurnNode::linear_move()
{
  //  FSM
  switch (state_) {
    case FORWARD:
      RCLCPP_INFO(get_logger(), "Moving forward!");
      l_vel_.linear.x = MOVE_SPEED;
      linear_->publish(l_vel_);

      if (check_distance()) {
        go_state(TURN);
      }
      break;

    case TURN:
      RCLCPP_INFO(get_logger(), "Rotating!");
      a_vel_.angular.z = TURN_SPEED;
      angular_->publish(a_vel_);

      if (check_turn()) {
        go_state(STOP);
      }
      break;

    case STOP:
      RCLCPP_INFO(get_logger(), "STOP!");
      l_vel_.linear.x = STOP_SPEED;
      a_vel_.angular.z = STOP_SPEED;
      break;
  }

}

void
ForwardTurnNode::go_state(int new_state)
{
  l_vel_.linear.x = STOP_SPEED;
  a_vel_.angular.z = STOP_SPEED;
  linear_->publish(l_vel_);
  angular_->publish(a_vel_);

  //  Change state
  state_ = new_state;
}

bool
ForwardTurnNode::check_distance()
{
  //  Check distance
  return distance_ >= 1.0;
}

bool
ForwardTurnNode::check_turn()
{
  //  Check angle
  return angle_ >= turn_limit_;
}

}  //  namespace forward_turn_cpp
