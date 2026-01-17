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

#include "talon_ros2_control/talon_hardware_interface.hpp"

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

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "talon_ros2_control/talon_control.h"
// #include "talon_control.h"

#define DEBUG_MODE 0 // 0 for off 1 for on

namespace talon_ros2_control
{
TALONHardwareInterface::TALONHardwareInterface() {
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info) // Info stores all parameters in xacro file
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  /*
  IF YOU WANT TO USE PARAMETERS FROM ROS2_CONTROL XACRO, DO THAT HERE!!! AS OF RIGHT NOW, NOT SURE
  IF NEEDED. USE info
  */

  for (auto& joint : info_.joints) {
    joint_node_ids.push_back(std::stoi(joint.parameters.at("node_id")));
    initial_position_.push_back(std::stof(joint.state_interfaces[0].initial_value));
  }

  num_joints = static_cast<int>(info_.joints.size());

  // Initializes command and state interface values
  joint_state_position_.assign(num_joints, 0);
  joint_state_velocity_.assign(num_joints, 0);

  joint_command_position_.assign(num_joints, 0);
  joint_command_velocity_.assign(num_joints, 0);

  control_level_.resize(num_joints, integration_level_t::POSITION);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  // Reuse cleanup logic which shuts down the motor and then deinitializes shared pointers.
  // Need this in case on_cleanup never gets called
  return this->on_cleanup(previous_state);
}

std::vector<hardware_interface::StateInterface> TALONHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for(int i = 0; i < num_joints; i++){
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_state_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_state_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TALONHardwareInterface::export_command_interfaces()
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

/*
* Setup communication with motor (init configs, send first CAN message)
*/
hardware_interface::CallbackReturn TALONHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Beginning configure.");

    for(int i = 0; i < num_joints; i++){
      TalonSRX *motor = new TalonSRX(joint_node_ids[i], "can0");
      talon_motors.push_back(motor);
      RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "talon_motor initialized.");
      initMotor(motor);
    }

    RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Configure complete.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TALONHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/) {

    for(int i = 0; i < num_joints; i++){
      stopMotor(talon_motors[i], 50);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

void TALONHardwareInterface::enable_system_thread() {
  while (is_running.load()) {
    ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

/*
* Activate system
*/
hardware_interface::CallbackReturn TALONHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Activating ...please wait...");

  is_running.store(true);
  worker = std::thread(&TALONHardwareInterface::enable_system_thread, this);

  // setPosition(talon_motor, 0.0, 50); // dangerous atm since this is an incremental encoder, dont want this to crush itself

  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
* Deactivate system
*/
hardware_interface::CallbackReturn TALONHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Deactivating ...please wait...");

  for(int i = 0; i < num_joints; i++){
    setVelocity(talon_motors[i], 0.0, 50);
  }

  is_running.store(false);
  if (worker.joinable()) {
    worker.join();
  }

  // setPosition(talon_motor, 0.0, 50); // dangerous atm since this is an incremental encoder, dont want this to crush itself

  RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type TALONHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for(int i = 0; i < num_joints; i++){
    joint_state_position_[i] = getPositionDistance(talon_motors[i]); // meters
    joint_state_velocity_[i] = getClawVelocity(talon_motors[i]); // m/s

    if(DEBUG_MODE == 1) {
      RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Reading for Talon - Joint position: %f Joint velocity: %f \n", 
                                                          joint_state_position_[i], 
                                                          joint_state_velocity_[i]);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type talon_ros2_control::TALONHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  float max_dist = 0.06; //in m
  for(int i = 0; i < num_joints; i++){
    if(control_level_[i] == integration_level_t::POSITION && std::isfinite(joint_command_position_[i])) {

      // TO DO: implement joint limits so i dont gotta do this
      if(joint_command_position_[i] > max_dist) { 
        joint_command_position_[i] = max_dist;
      }
      else if(joint_command_position_[i] < 0.0) {
        joint_command_position_[i] = 0.0;
      }
      
      // COMMAND POSITION
      setPosition(talon_motors[i], joint_command_position_[i], 50);

      if(DEBUG_MODE == 1) {
        RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Writing positions for Talon - Joint command: %f",
                                                              joint_command_position_[i]);
      }
    } else if(control_level_[i] == integration_level_t::VELOCITY && std::isfinite(joint_command_velocity_[i])) {

      // COMMAND VELOCITY
      setVelocity(talon_motors[i], joint_command_velocity_[i], 50);

      if(DEBUG_MODE == 1) {
        RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Writing velocities for Talon - Joint command: %f",
                                                              joint_command_velocity_[i]);
      }
    } else {
      // RCLCPP_INFO(rclcpp::get_logger("TALONHardwareInterface"), "Joint command value not found or undefined command state");
    }
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TALONHardwareInterface::perform_command_mode_switch(
  const std::vector<std::string>& start_interfaces,
  const std::vector<std::string>& stop_interfaces)
{
  std::vector<integration_level_t> new_modes = {};
  for (std::string key : start_interfaces)
  {
    for (int i = 0; i < num_joints; i++){
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION){
        new_modes.push_back(integration_level_t::POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY){
        new_modes.push_back(integration_level_t::VELOCITY);
      }
    }
  }
  
  // All joints must be given new command mode at the same time
  if (new_modes.size() != static_cast<unsigned long>(num_joints)){
    return hardware_interface::return_type::ERROR;
  }
  // All joints must have the same command mode
  if (!std::all_of(
        new_modes.begin() + 1, new_modes.end(),
        [&](integration_level_t mode) { return mode == new_modes[0]; }))
  {
    return hardware_interface::return_type::ERROR;
  }

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces) {
    for (int i = 0; i < num_joints; i++){
      if (key.find(info_.joints[i].name) != std::string::npos) {
        joint_command_position_[i] = 0;
        joint_command_velocity_[i] = 0;
        control_level_[i] = integration_level_t::UNDEFINED;  // Revert to undefined
      }
    }
  }
  // Set the new command modes. By this point everything should be undefined after the "stop motion" loop
  for (int i = 0; i < num_joints; i++){
    if (control_level_[i] != integration_level_t::UNDEFINED) {
      // Something else is using the joint! Abort!
      return hardware_interface::return_type::ERROR;
    }
    control_level_[i] = new_modes[i];
  }

  return hardware_interface::return_type::OK;
}

}  // namespace talon_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  talon_ros2_control::TALONHardwareInterface, hardware_interface::SystemInterface)
