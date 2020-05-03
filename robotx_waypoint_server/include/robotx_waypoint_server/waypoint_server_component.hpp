// Copyright (c) 2020, OUXT-Polaris
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

#ifndef ROBOTX_WAYPOINT_SERVER__WAYPOINT_SERVER_HPP_
#define ROBOTX_WAYPOINT_SERVER__WAYPOINT_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <memory>
#include <cmath>

#include "robotx_waypoint_server/visibility.h"
#include "robotx_waypoint_msgs/action/way_point.hpp"

namespace robotx_waypoint_server
{
using WayPoint = robotx_waypoint_msgs::action::WayPoint;
using geometry_msgs::msg::PoseStamped;

class WaypointServerComponent : public rclcpp::Node
{
public:
  ROBOTX_WAYPOINT_SERVER_PUBLIC WaypointServerComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("waypoint_server", options)
  {
    using namespace std::placeholders;  // NOLINT
    waypoint_pub_ = create_publisher<PoseStamped>("move_base_simple/goal", 1);
    current_pose_sub_ = create_subscription<PoseStamped>(
      "current_pose", 1,
      std::bind(&WaypointServerComponent::callbackCurrentPose, this, _1));

    action_server_ = rclcpp_action::create_server<WayPoint>(
      node_->get_node_base_interface(),
      node_->get_node_clock_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(),
      "waypoint",
      std::bind(&WaypointServerComponent::handle_goal, this, _1, _2),
      std::bind(&WaypointServerComponent::handle_cancel, this, _1),
      std::bind(&WaypointServerComponent::handle_accepted, this, _1));

    using std::chrono_literals::operator""ms;

    timer_ = create_wall_timer(
      500ms, std::bind(&WaypointServerComponent::timerCallback, this));
    std::cout << "TIMER CREATED" << std::endl;
  }

  void timerCallback()
  {
    RCLCPP_INFO(this->get_logger(), "tick");
  }

  void publishWayPoint(geometry_msgs::msg::Pose pose)
  {
    PoseStamped pose_stamped;
    pose_stamped.pose = pose;
    pose_stamped.header.set__frame_id("map");
    pose_stamped.header.set__stamp(node_->get_clock()->now());
    waypoint_pub_->publish(pose_stamped);
  }

private:
  void callbackCurrentPose(const PoseStamped::SharedPtr pose)
  {
    current_pose_ = *pose;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const WayPoint::Goal> goal)
  {}

  rclcpp_action::CancelResponse handle_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<WayPoint>> goal_handle)
  {}

  rclcpp_action::GoalResponse handle_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<WayPoint>> goal_handle)
  {
    using namespace std::placeholders;  // NOLINT
    std::thread{std::bind(&WaypointServerComponent::execute, this, _1), goal_handle}.detach();
    RCLCPP_INFO(node_->get_logger(), "action server started!");
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<WayPoint>> goal_handle)
  {
    rclcpp::Rate loop_rate(10);
    auto feedback = std::make_shared<WayPoint::Feedback>();
    auto result = std::make_shared<WayPoint::Result>();

    while (rclcpp::ok()) {
      // check cancel
      if (goal_handle->is_canceling()) {
        result->is_reached.set__data(false);
        goal_handle->canceled(result);
        RCLCPP_INFO(node_->get_logger(), "Canceled");
        return;
      }
      // check reached
      const auto goal = goal_handle->get_goal();
      float squared_distance =
        std::pow(goal->pose.position.x - current_pose_.pose.position.x, 2) + std::pow(
        goal->pose.position.y - current_pose_.pose.position.y, 2);
      if (squared_distance < std::pow(goal->threshold.data, 2)) {
        result->is_reached.set__data(true);
        goal_handle->succeed(result);
        return;
      }
      // feedback
      feedback->pose_stamped = current_pose_;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(node_->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }
  }
  rclcpp::Publisher<PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr current_pose_sub_;
  PoseStamped current_pose_;
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Server<WayPoint>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace robotx_waypoint_server
#endif  // ROBOTX_WAYPOINT_SERVER__WAYPOINT_SERVER_HPP_
