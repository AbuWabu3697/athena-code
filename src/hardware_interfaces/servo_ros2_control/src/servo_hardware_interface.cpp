// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
// Authors: Denis Stogl
//

#include "servo_ros2_control/servo_hardware_interface.hpp"

#include <netdb.h>
#include <sys/socket.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <cstring>
#include <sstream>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <cmath>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace servo_ros2_control
{
hardware_interface::CallbackReturn SERVOHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info) // Info stores all parameters in xacro file
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  /*
  IF YOU WANT TO USE PARAMETERS FROM ROS2_CONTROL XACRO, DO THAT HERE!!!
  */
  
  // This stores the can ids for each joint aka motor
  /* for (auto& joint : info_.joints) {
    joint_node_ids.push_back(std::stoi(joint.parameters.at("node_id")));
  } */

  num_joints = static_cast<int>(info_.joints.size());
  
  // Initializes command and state interface values
  joint_state_position_.assign(num_joints, 0);
  joint_state_velocity_.assign(num_joints, 0);

  joint_command_position_.assign(num_joints, 0);
  joint_command_velocity_.assign(num_joints, 0);

  encoder_position = 0;
  motor_speed = 0;

  control_level_.resize(num_joints, integration_level_t::UNDEFINED);

  node_ = rclcpp::Node::make_shared("servo_hardware_node");
  actuator_can_publisher_ = node_->create_publisher<msgs::msg::CANA>("can_tx", 10);

  // Lambda function that takes the message as a shared pointer, dereferences it, 
  // and stores it in received_joint_data_ to be used
  actuator_can_subscriber_ = node_->create_subscription<msgs::msg::CANA>(
      "can_rx", 
      10, 
      [this](const msgs::msg::CANA::SharedPtr received_message) 
      {
        received_joint_data_ = *received_message;
      });

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SERVOHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // may not need this either
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> SERVOHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Each SERVO motor corresponds to a different joint.
  for(int i = 0; i < num_joints; i++){   
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_position_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));
  }
  
  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
SERVOHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for(int i = 0; i < num_joints; i++){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_command_position_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_command_velocity_[i]));
  }

  return command_interfaces;
}


hardware_interface::CallbackReturn SERVOHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Activating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SERVOHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto joint_tx = msgs::msg::CANA();
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "Successfully deactivated all SERVO motors!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type SERVOHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type servo_ros2_control::SERVOHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SERVOHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  // Debug: print incoming requests
  std::ostringstream ss;
  ss << "perform_command_mode_switch called. start_interfaces: [";
  for (auto &s : start_interfaces) ss << s << ",";
  ss << "] stop_interfaces: [";
  for (auto &s : stop_interfaces) ss << s << ",";
  ss << "]";
  RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"), "%s", ss.str().c_str());

  // For each joint, decide its new control mode based on start/stop interfaces.
  std::vector<integration_level_t> requested_modes(num_joints, integration_level_t::UNDEFINED);

  // -------- STOP INTERFACES --------
  for (const auto &ifname : stop_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);

      if (ifname == pos_if || ifname == vel_if || ifname.find(info_.joints[i].name) != std::string::npos) {
        requested_modes[i] = integration_level_t::UNDEFINED;
      }
    }
  }

  // -------- START INTERFACES --------
  for (const auto &ifname : start_interfaces) {
    for (int i = 0; i < num_joints; ++i) {
      const std::string pos_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_POSITION);
      const std::string vel_if = info_.joints[i].name + "/" + std::string(hardware_interface::HW_IF_VELOCITY);

      if (ifname == pos_if) {
        requested_modes[i] = integration_level_t::POSITION;
      } else if (ifname == vel_if) {
        requested_modes[i] = integration_level_t::VELOCITY;
      }
    }
  }

  // -------- APPLY MODES --------
  for (int i = 0; i < num_joints; ++i) {

    if (requested_modes[i] == integration_level_t::UNDEFINED) {
      bool was_stopped = false;
      for (const auto &ifname : stop_interfaces) {
        if (ifname.find(info_.joints[i].name) != std::string::npos) {
          was_stopped = true;
          break;
        }
      }

      if (was_stopped) {
        control_level_[i] = integration_level_t::UNDEFINED;
        joint_command_velocity_[i] = 0;
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"),
          "Joint %s: stopped -> UNDEFINED",
          info_.joints[i].name.c_str());
      }

    } else {

      control_level_[i] = requested_modes[i];

      if (requested_modes[i] == integration_level_t::VELOCITY) {
        joint_command_velocity_[i] = 0.0;
        RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"),
          "Joint %s: switched to VELOCITY",
          info_.joints[i].name.c_str());
      } else if (requested_modes[i] == integration_level_t::POSITION) {
        joint_command_position_[i] = joint_state_position_[i];
        RCLCPP_INFO(rclcpp::get_logger("SERVOHardwareInterface"),
          "Joint %s: switched to POSITION (init pos=%f)",
          info_.joints[i].name.c_str(), joint_command_position_[i]);
      }
    }
  }

  return hardware_interface::return_type::OK;
}


}  // namespace servo_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  servo_ros2_control::SERVOHardwareInterface, hardware_interface::SystemInterface)
