// Copyright (c) 2023 Franka Robotics GmbH
// Copyright (c) 2026 Dimas Abreu Archanjo Dutra
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
//
// This file is a modified version of franka_example_controllers'
// src/fr3/cartesian_pose_example_controller.cpp (frankarobotics/franka_ros2, v3.4.0),
// changed to track a setpoint received on the "ee_pose_setpoint" topic instead of
// following a fixed periodic motion.

#include <fr3_motion/cartesian_pose_controller.hpp>
#include <fr3_motion/setpoint_utils.hpp>

#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

namespace fr3_motion {

controller_interface::InterfaceConfiguration
CartesianPoseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  // add the robot time interface
  config.names.push_back(arm_prefix_ + robot_type_ + "/robot_time");
  return config;
}

controller_interface::return_type CartesianPoseController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    // Get initial orientation and translation, in case no setpoint has been received yet.
    std::tie(orientation_setpoint_, position_setpoint_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    setpoint_buffer_.writeFromNonRT(Setpoint{position_setpoint_, orientation_setpoint_});
    initialization_flag_ = false;
  }

  const auto setpoint = *setpoint_buffer_.readFromRT();
  position_setpoint_ = setpoint.position;
  orientation_setpoint_ = setpoint.orientation;

  if (franka_cartesian_pose_->setCommand(orientation_setpoint_, position_setpoint_)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn CartesianPoseController::on_init() {
  auto_declare<std::string>("arm_prefix", "");
  auto_declare<double>("setpoint_timeout", std::numeric_limits<double>::infinity());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(arm_prefix_,
                                                                   k_elbow_activated_));

  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  robot_type_ =
      robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  latest_setpoint_stamp_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  ee_pose_setpoint_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "ee_pose_setpoint", rclcpp::SystemDefaultsQoS(),
      std::bind(&CartesianPoseController::setpointCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianPoseController::setpointCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  const auto& logger = get_node()->get_logger();
  const rclcpp::Time new_stamp(msg->header.stamp, get_node()->get_clock()->get_clock_type());
  const rclcpp::Time now = get_node()->get_clock()->now();
  const double timeout = get_node()->get_parameter("setpoint_timeout").as_double();

  if (!fr3_motion::accept_setpoint(new_stamp, latest_setpoint_stamp_, now, timeout, logger,
                                   "end-effector pose")) {
    return;
  }

  Setpoint setpoint;
  setpoint.position =
      Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  setpoint.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                            msg->pose.orientation.y, msg->pose.orientation.z);

  latest_setpoint_stamp_ = new_stamp;
  initialization_flag_ = false;
  setpoint_buffer_.writeFromNonRT(setpoint);
}

}  // namespace fr3_motion
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_motion::CartesianPoseController,
                       controller_interface::ControllerInterface)
