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
// include/franka_example_controllers/fr3/joint_impedance_example_controller.hpp
// (frankarobotics/franka_ros2, v3.4.0), changed to track a setpoint received on the
// "joint_state_setpoint" topic instead of following a fixed periodic motion.

#pragma once

#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_motion {

/**
 * Joint impedance controller that tracks a joint position setpoint received on the
 * "joint_state_setpoint" topic, instead of the fixed periodic motion of
 * franka_example_controllers::JointImpedanceExampleController it is based on.
 */
class JointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string robot_type_;
  std::string arm_prefix_;
  std::string robot_description_;
  const int num_joints = 7;
  std::vector<std::string> joint_names_;
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  void updateJointStates();

  // Setpoint handling: q_setpoint_ takes the place of the example controller's q_goal,
  // but is driven by the "joint_state_setpoint" topic instead of a fixed motion.
  Vector7d q_setpoint_;
  realtime_tools::RealtimeBuffer<Vector7d> q_setpoint_buffer_;
  rclcpp::Time latest_setpoint_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_setpoint_sub_;
  void setpointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace fr3_motion
