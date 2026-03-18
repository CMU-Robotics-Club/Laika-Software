cmake_minimum_required(VERSION 3.8)
project(laika_cartesian_impedance_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

######### Set Dependencies
set(DEPENDENCIES
  ament_cmake
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  generate_parameter_library
  control_toolbox
  control_msgs
  realtime_tools
)

######### Find Dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

######### Laika Cartesian Impedance Controller
generate_parameter_library(laika_cartesian_impedance_controller_parameters controller/laika_cartesian_impedance_controller_parameters.yaml)
add_library(
  laika_cartesian_impedance_controller
  SHARED
  controller/laika_cartesian_impedance_controller.cpp
)
target_compile_features(laika_cartesian_impedance_controller PUBLIC cxx_std_20)
target_include_directories(laika_cartesian_impedance_controller PRIVATE controller/include)
target_link_libraries(laika_cartesian_impedance_controller PUBLIC laika_cartesian_impedance_controller_parameters)
ament_target_dependencies(
  laika_cartesian_impedance_controller 
  PUBLIC
  controller_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  control_toolbox
  control_msgs
  realtime_tools
  std_msgs
)
pluginlib_export_plugin_description_file(controller_interface laika_cartesian_impedance_controller.xml)
install(TARGETS laika_cartesian_impedance_controller laika_cartesian_impedance_controller_parameters EXPORT export_laika_cartesian_impedance_controller)
install(TARGETS laika_cartesian_impedance_controller EXPORT export_laika_cartesian_impedance_controller)
install(DIRECTORY controller/include DESTINATION share/laika_cartesian_impedance_controller)

######### Install
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
