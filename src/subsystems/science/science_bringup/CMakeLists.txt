cmake_minimum_required(VERSION 3.8)
project(science_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_runtime REQUIRED)


install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

