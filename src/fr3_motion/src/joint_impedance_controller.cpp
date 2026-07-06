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
// src/fr3/joint_impedance_example_controller.cpp (frankarobotics/franka_ros2, v3.4.0),
// changed to track a setpoint received on the "joint_state_setpoint" topic instead of
// following a fixed periodic motion.

#include <fr3_motion/joint_impedance_controller.hpp>
#include <fr3_motion/setpoint_utils.hpp>

#include <franka_example_controllers/robot_utils.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

#include <Eigen/Eigen>

namespace fr3_motion {

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  q_setpoint_ = *q_setpoint_buffer_.readFromRT();

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_setpoint_ - q_) + d_gains_.cwiseProduct(-dq_filtered_);

  for (int i = 0; i < num_joints; ++i) {
    if (!command_interfaces_[i].set_value(tau_d_calculated(i))) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set command interface value");
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "");
    auto_declare<std::string>("arm_prefix", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("setpoint_timeout", std::numeric_limits<double>::infinity());
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  joint_names_.clear();
  for (int i = 1; i <= num_joints; ++i) {
    joint_names_.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i));
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

  latest_setpoint_stamp_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());
  joint_state_setpoint_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      "joint_state_setpoint", rclcpp::SystemDefaultsQoS(),
      std::bind(&JointImpedanceController::setpointCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_filtered_.setZero();
  q_setpoint_ = q_;
  q_setpoint_buffer_.writeFromNonRT(q_setpoint_);

  return CallbackReturn::SUCCESS;
}

void JointImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_optional().value();
    dq_(i) = velocity_interface.get_optional().value();
  }
}

void JointImpedanceController::setpointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  const auto& logger = get_node()->get_logger();
  const rclcpp::Time new_stamp(msg->header.stamp, get_node()->get_clock()->get_clock_type());
  const rclcpp::Time now = get_node()->get_clock()->now();
  const double timeout = get_node()->get_parameter("setpoint_timeout").as_double();

  if (!fr3_motion::accept_setpoint(new_stamp, latest_setpoint_stamp_, now, timeout, logger,
                                   "joint state")) {
    return;
  }

  Vector7d new_setpoint;
  if (!msg->name.empty()) {
    for (int i = 0; i < num_joints; ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
      if (it == msg->name.end()) {
        RCLCPP_INFO(logger, "Dropping joint state setpoint: missing joint name '%s'.",
                    joint_names_[i].c_str());
        return;
      }
      new_setpoint(i) = msg->position.at(static_cast<size_t>(it - msg->name.begin()));
    }
  } else {
    if (msg->position.size() != static_cast<size_t>(num_joints)) {
      RCLCPP_INFO(logger,
                  "Dropping joint state setpoint: expected %d positions (or named joints), got "
                  "%zu.",
                  num_joints, msg->position.size());
      return;
    }
    for (int i = 0; i < num_joints; ++i) {
      new_setpoint(i) = msg->position[static_cast<size_t>(i)];
    }
  }

  latest_setpoint_stamp_ = new_stamp;
  q_setpoint_buffer_.writeFromNonRT(new_setpoint);
}

}  // namespace fr3_motion
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_motion::JointImpedanceController,
                       controller_interface::ControllerInterface)
