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
// src/fr3/joint_impedance_with_ik_example_controller.cpp
// (frankarobotics/franka_ros2, v3.4.0), changed to track a setpoint received on the
// "ee_pose_setpoint" topic instead of following a fixed periodic motion.

#include <fr3_motion/joint_impedance_with_ik_controller.hpp>
#include <fr3_motion/setpoint_utils.hpp>

#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <limits>
#include <rclcpp/logging.hpp>
#include <string>

#include <chrono>

using namespace std::chrono_literals;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace fr3_motion {

static constexpr std::chrono::duration<double> kTimeStep{1s};
static constexpr std::chrono::duration<double> kMaxWaitingTime{10s};

controller_interface::InterfaceConfiguration
JointImpedanceWithIKController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceWithIKController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_prefix_ + robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }

  return config;
}

void JointImpedanceWithIKController::update_joint_states() {
  for (auto i = 0; i < num_joints_; ++i) {
    // TODO(yazi_ba) Can we get the state from its name?
    const auto& position_interface = state_interfaces_.at(16 + i);
    const auto& velocity_interface = state_interfaces_.at(23 + i);
    const auto& effort_interface = state_interfaces_.at(30 + i);

    joint_positions_current_[i] = position_interface.get_optional().value();
    joint_velocities_current_[i] = velocity_interface.get_optional().value();
    joint_efforts_current_[i] = effort_interface.get_optional().value();
  }
}

std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request>
JointImpedanceWithIKController::create_ik_service_request(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::vector<double>& joint_positions_current,
    const std::vector<double>& joint_velocities_current,
    const std::vector<double>& joint_efforts_current) {
  auto service_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

  service_request->ik_request.group_name = arm_prefix_ + robot_type_ + "_arm";
  service_request->ik_request.pose_stamped.header.frame_id = arm_prefix_ + robot_type_ + "_link0";
  service_request->ik_request.pose_stamped.pose.position.x = position.x();
  service_request->ik_request.pose_stamped.pose.position.y = position.y();
  service_request->ik_request.pose_stamped.pose.position.z = position.z();
  service_request->ik_request.pose_stamped.pose.orientation.x = orientation.x();
  service_request->ik_request.pose_stamped.pose.orientation.y = orientation.y();
  service_request->ik_request.pose_stamped.pose.orientation.z = orientation.z();
  service_request->ik_request.pose_stamped.pose.orientation.w = orientation.w();
  service_request->ik_request.robot_state.joint_state.name = {
      arm_prefix_ + robot_type_ + "_joint1", arm_prefix_ + robot_type_ + "_joint2",
      arm_prefix_ + robot_type_ + "_joint3", arm_prefix_ + robot_type_ + "_joint4",
      arm_prefix_ + robot_type_ + "_joint5", arm_prefix_ + robot_type_ + "_joint6",
      arm_prefix_ + robot_type_ + "_joint7"};
  service_request->ik_request.robot_state.joint_state.position = joint_positions_current;
  service_request->ik_request.robot_state.joint_state.velocity = joint_velocities_current;
  service_request->ik_request.robot_state.joint_state.effort = joint_efforts_current;

  if (is_gripper_loaded_) {
    service_request->ik_request.ik_link_name = arm_prefix_ + robot_type_ + "_hand_tcp";
  }
  return service_request;
}

Vector7d JointImpedanceWithIKController::compute_torque_command(
    const Vector7d& joint_positions_desired,
    const Vector7d& joint_positions_current,
    const Vector7d& joint_velocities_current) {
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d coriolis(coriolis_array.data());
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current;
  Vector7d q_error = joint_positions_desired - joint_positions_current;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_) + coriolis;

  return tau_d_calculated;
}

controller_interface::return_type JointImpedanceWithIKController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    // Get initial orientation and translation, in case no setpoint has been received yet.
    std::tie(orientation_setpoint_, position_setpoint_) =
        franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    setpoint_buffer_.writeFromNonRT(Setpoint{position_setpoint_, orientation_setpoint_});
    initialization_flag_ = false;
  }
  update_joint_states();

  const auto setpoint = *setpoint_buffer_.readFromRT();
  position_setpoint_ = setpoint.position;
  orientation_setpoint_ = setpoint.orientation;

  auto service_request =
      create_ik_service_request(position_setpoint_, orientation_setpoint_, joint_positions_current_,
                                joint_velocities_current_, joint_efforts_current_);

  using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture;
  auto response_received_callback =
      [&](ServiceResponseFuture future) {  // NOLINT(performance-unnecessary-value-param)
        const auto& response = future.get();

        if (response->error_code.val == response->error_code.SUCCESS) {
          joint_positions_desired_ = response->solution.joint_state.position;
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Inverse kinematics solution failed.");
        }
      };
  auto result_future_ =
      compute_ik_client_->async_send_request(service_request, response_received_callback);

  if (joint_positions_desired_.empty()) {
    return controller_interface::return_type::OK;
  }

  Vector7d joint_positions_desired_eigen(joint_positions_desired_.data());
  Vector7d joint_positions_current_eigen(joint_positions_current_.data());
  Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());

  auto tau_d_calculated = compute_torque_command(
      joint_positions_desired_eigen, joint_positions_current_eigen, joint_velocities_current_eigen);
  for (int i = 0; i < num_joints_; i++) {
    if (!command_interfaces_[i].set_value(tau_d_calculated(i))) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface value");
      return controller_interface::return_type::ERROR;
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceWithIKController::on_init() {
  auto_declare<std::string>("robot_type", "fr3");
  auto_declare<std::string>("arm_prefix", "");
  auto_declare("load_gripper", false);
  auto_declare<double>("setpoint_timeout", std::numeric_limits<double>::infinity());
  std::vector<double> default_k_gains{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0};
  std::vector<double> default_d_gains{30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0};
  auto_declare("k_gains", default_k_gains);
  auto_declare("d_gains", default_d_gains);

  return CallbackReturn::SUCCESS;
}

bool JointImpedanceWithIKController::assign_parameters() {
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();
  arm_prefix_ = arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(arm_prefix_,
                                                                   k_elbow_activated_));

  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  is_gripper_loaded_ = get_node()->get_parameter("load_gripper").as_bool();

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return false;
  }
  if (k_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints_, k_gains.size());
    return false;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return false;
  }
  if (d_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints_, d_gains.size());
    return false;
  }
  for (int i = 0; i < num_joints_; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  return true;
}

CallbackReturn JointImpedanceWithIKController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(
          arm_prefix_ + robot_type_ + "/" + k_robot_model_interface_name,
          arm_prefix_ + robot_type_ + "/" + k_robot_state_interface_name));

  auto collision_client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  compute_ik_client_ = get_node()->create_client<moveit_msgs::srv::GetPositionIK>("compute_ik");

  auto timer = std::chrono::duration<double>::zero();
  while (!compute_ik_client_->wait_for_service(kTimeStep) ||
         !collision_client->wait_for_service(kTimeStep)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_node()->get_logger(),
                "IK service not available, waited for %f seconds, waiting more...", timer.count());
    timer += kTimeStep;
    if (timer > kMaxWaitingTime) {
      RCLCPP_FATAL(get_node()->get_logger(),
                   "Could not connect to IK service - did you start move_group (e.g. "
                   "fr3_motion/launch/move_group.launch.py)?");
      return CallbackReturn::ERROR;
    }
  }

  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
  auto future_result = collision_client->async_send_request(request);

  auto success = future_result.get();

  if (!success->success) {
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
      std::bind(&JointImpedanceWithIKController::setpointCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceWithIKController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  dq_filtered_.setZero();
  joint_positions_desired_.reserve(num_joints_);
  joint_positions_current_.reserve(num_joints_);
  joint_velocities_current_.reserve(num_joints_);
  joint_efforts_current_.reserve(num_joints_);

  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointImpedanceWithIKController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void JointImpedanceWithIKController::setpointCallback(
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
PLUGINLIB_EXPORT_CLASS(fr3_motion::JointImpedanceWithIKController,
                       controller_interface::ControllerInterface)
