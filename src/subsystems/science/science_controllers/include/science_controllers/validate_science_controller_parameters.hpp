// Copyright (c) 2025, UMDLoop
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#ifndef SCIENCE_CONTROLLERS__VALIDATE_SCIENCE_CONTROLLER_PARAMETERS_HPP_
#define SCIENCE_CONTROLLERS__VALIDATE_SCIENCE_CONTROLLER_PARAMETERS_HPP_

#include <string>
#include "parameter_traits/parameter_traits.hpp"

#include "rclcpp/rclcpp.hpp" 

namespace parameter_traits 
{
inline parameter_traits::Result forbidden_interface_name_prefix(rclcpp::Parameter const & parameter)
{
  auto const & interface_name = parameter.as_string();

  if (interface_name.rfind("blup_", 0) == 0)
  {
    return parameter_traits::ERROR("'interface_name' parameter can not start with 'blup_'");
  }

  return parameter_traits::OK;
}

}  // namespace parameter_traits

#endif  // SCIENCE_CONTROLLERS__VALIDATE_SCIENCE_MANUAL_PARAMETERS_HPP_
