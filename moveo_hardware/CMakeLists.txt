cmake_minimum_required(VERSION 3.8)
project(moveo_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(moveo_hardware SHARED
  src/moveo_system.cpp
)

# THIS IS THE IMPORTANT PART
target_include_directories(moveo_hardware PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(moveo_hardware
  rclcpp
  hardware_interface
  pluginlib
)

install(TARGETS moveo_hardware
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(
  hardware_interface
  moveo_hardware_plugin.xml
)

ament_package()