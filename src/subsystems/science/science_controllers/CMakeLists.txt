cmake_minimum_required(VERSION 3.8)
project(science_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow -Wno-shadow) 
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add science_manual library related compile commands
generate_parameter_library(
  science_manual_parameters
  src/science_controller.yaml
  include/science_controllers/validate_science_controller_parameters.hpp
)

add_library(
  science_manual
  SHARED
  src/science_controller.cpp
)
target_include_directories(science_manual PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(science_manual science_manual_parameters)
ament_target_dependencies(science_manual ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(science_manual PRIVATE "SCIENCE_MANUAL_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface science_controllers.xml)

install(
  TARGETS
  science_manual
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/
  DESTINATION include/${PROJECT_NAME}
)


# if(BUILD_TESTING)

#   ament_add_gmock(test_load_athena_science_manual test/test_load_athena_science_manual.cpp)
#   target_include_directories(test_load_athena_science_manual PRIVATE include)
#   ament_target_dependencies(
#     test_load_athena_science_manual
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   add_rostest_with_parameters_gmock(test_athena_science_manual test/test_athena_science_manual.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/athena_science_manual_params.yaml)
#   target_include_directories(test_athena_science_manual PRIVATE include)
#   target_link_libraries(test_athena_science_manual athena_science_manual)
#   ament_target_dependencies(
#     test_athena_science_manual
#     controller_interface
#     hardware_interface
#   )

#   add_rostest_with_parameters_gmock(test_athena_science_manual_preceeding test/test_athena_science_manual_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/athena_science_manual_preceeding_params.yaml)
#   target_include_directories(test_athena_science_manual_preceeding PRIVATE include)
#   target_link_libraries(test_athena_science_manual_preceeding athena_science_manual)
#   ament_target_dependencies(
#     test_athena_science_manual_preceeding
#     controller_interface
#     hardware_interface
#   )
# endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  science_manual
)

ament_package()