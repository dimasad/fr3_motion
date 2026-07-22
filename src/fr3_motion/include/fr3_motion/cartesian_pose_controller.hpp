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
// include/franka_example_controllers/fr3/cartesian_pose_example_controller.hpp
// (frankarobotics/franka_ros2, v3.4.0), changed to track a setpoint received on the
// "ee_pose_setpoint" topic instead of following a fixed periodic motion.

#pragma once

#include <array>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_motion {

/**
 * Cartesian pose controller that tracks an end-effector pose setpoint received on the
 * "ee_pose_setpoint" topic, instead of the fixed periodic motion of
 * franka_example_controllers::CartesianPoseExampleController it is based on.
 */
class CartesianPoseController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  std::string robot_description_;
  std::string robot_type_;
  std::string arm_prefix_;

  // Setpoint handling: position_setpoint_/orientation_setpoint_ take the place of the
  // example controller's position_/orientation_, but are driven by the "ee_pose_setpoint"
  // topic instead of a fixed motion.
  struct Setpoint {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
  };
  Eigen::Vector3d position_setpoint_;
  Eigen::Quaterniond orientation_setpoint_;
  realtime_tools::RealtimeBuffer<Setpoint> setpoint_buffer_;
  rclcpp::Time latest_setpoint_stamp_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_setpoint_sub_;
  void setpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Cartesian pose commands go through libfranka's motion generator, which requires the
  // commanded pose to be continuous every 1 kHz control cycle (bounded velocity, acceleration
  // and jerk). Setpoints arriving from "ee_pose_setpoint" are not guaranteed to satisfy that on
  // their own (e.g. a lower-rate publisher holds a pose for several cycles, then jumps), so
  // every commanded pose is passed through franka::limitRate before being sent to the hardware.
  // last_pose_command_/last_twist_command_/last_acceleration_command_ hold the rate limiter's
  // own state across cycles, the same way dq_filtered_ persists across cycles in
  // JointImpedanceController.
  double max_translational_velocity_{};
  double max_translational_acceleration_{};
  double max_translational_jerk_{};
  double max_rotational_velocity_{};
  double max_rotational_acceleration_{};
  double max_rotational_jerk_{};

  std::array<double, 16> last_pose_command_{};
  std::array<double, 6> last_twist_command_{};
  std::array<double, 6> last_acceleration_command_{};
};

}  // namespace fr3_motion
