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
  hardware/quads_usb_protocol.cpp
  hardware/usb_serial.cpp
  hardware/laika_usb_hardware_interface.cpp
)
target_compile_features(laika_hardware_interface PUBLIC cxx_std_20)
target_include_directories(laika_hardware_interface PRIVATE hardware/include)
target_include_directories(laika_hardware_interface PRIVATE odrive_base/include)
# Modern CMake targets: ament_target_dependencies is gone in Lyrical, these work in Jazzy too
target_link_libraries(
  laika_hardware_interface
  PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  hardware_interface::hardware_interface
  pluginlib::pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface laika_hardware_interface.xml)
install(TARGETS laika_hardware_interface EXPORT export_laika_hardware_interface)
install(DIRECTORY hardware/include DESTINATION share/laika_hardware_interface)
install(DIRECTORY odrive_base/include DESTINATION share/laika_hardware_interface)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

######### Tests (colcon test --packages-select laika_hardware_interface)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Framing only: compiled on its own, no ROS needed
  ament_add_gtest(test_quads_usb_protocol
    test/test_quads_usb_protocol.cpp
    hardware/quads_usb_protocol.cpp
  )
  target_compile_features(test_quads_usb_protocol PUBLIC cxx_std_20)
  target_include_directories(test_quads_usb_protocol PRIVATE hardware/include)

  # Whole plugin against a pseudo-terminal instead of the STM32
  ament_add_gtest(test_laika_usb_hardware_interface
    test/test_laika_usb_hardware_interface.cpp
    hardware/laika_usb_hardware_interface.cpp
    hardware/usb_serial.cpp
    hardware/quads_usb_protocol.cpp
  )
  target_compile_features(test_laika_usb_hardware_interface PUBLIC cxx_std_20)
  target_include_directories(test_laika_usb_hardware_interface PRIVATE hardware/include)
  target_link_libraries(test_laika_usb_hardware_interface
    hardware_interface::hardware_interface rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle pluginlib::pluginlib
    util)  # util: openpty
endif()

ament_package()
