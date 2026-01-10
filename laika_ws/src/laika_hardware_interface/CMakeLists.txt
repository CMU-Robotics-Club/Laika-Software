cmake_minimum_required(VERSION 3.8)
project(laika_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

######### Set Dependencies
set(DEPENDENCIES
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

######### Find Dependencies
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()
ament_export_dependencies(${DEPENDENCIES})

######### ODrive Hardware Interface
add_library(
  laika_hardware_interface
  SHARED
  odrive_base/src/socket_can.cpp
  odrive_base/src/epoll_event_loop.cpp
  hardware/laika_hardware_interface.cpp
)
target_compile_features(laika_hardware_interface PUBLIC cxx_std_20)
target_include_directories(laika_hardware_interface PRIVATE hardware/include)
target_include_directories(laika_hardware_interface PRIVATE odrive_base/include)
ament_target_dependencies(
  laika_hardware_interface 
  PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface laika_hardware_interface.xml)
install(TARGETS laika_hardware_interface EXPORT export_laika_hardware_interface)
install(DIRECTORY hardware/include DESTINATION share/laika_hardware_interface)
install(DIRECTORY odrive_base/include DESTINATION share/laika_hardware_interface)

ament_package()
