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

#ifndef SERVO_HARDWARE_INTERACE_HPP_
#define SERVO_HARDWARE_INTERACE_HPP_

#include <netinet/in.h>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>


#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include "msgs/msg/cana.hpp"

namespace servo_ros2_control
{
class SERVOHardwareInterface : public hardware_interface::SystemInterface // Inheriting from System Interface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SERVOHardwareInterface)

  // Initialization, so reading parameters, initializing variables, checking if all the joint state and command interfaces are correct
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // Exports/exposes Interfaces that are available so that the controllers
  // know what to read and write to
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Establish communications and set initial values for state and command interfaces
  // hardware_interface::CallbackReturn on_configure(
  //   const rclcpp_lifecycle::State & previous_state) override;

  // Lifecycle
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces
  ) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  int num_joints;

  // Store the state for the simulated robot
  std::vector<double> joint_state_position_;
  std::vector<double> joint_state_velocity_;
  
  // Store the command for the simulated robot
  std::vector<double> joint_command_position_;
  std::vector<double> joint_command_velocity_;

  double encoder_position;
  double motor_speed;

  rclcpp::Publisher<msgs::msg::CANA>::SharedPtr actuator_can_publisher_;
  rclcpp::Subscription<msgs::msg::CANA>::SharedPtr actuator_can_subscriber_;
  rclcpp::Node::SharedPtr node_;

  msgs::msg::CANA received_joint_data_;

  std::vector<int> joint_node_ids;
  std::vector<int> joint_gear_ratios;


  enum integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
  };

  // Active control mode for each servok
  std::vector<integration_level_t> control_level_;

};

}  // namespace servo_hardware_interface

#endif  // SERVO_HARDWARE_INTERACE_HPP_

