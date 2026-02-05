cmake_minimum_required(VERSION 3.8)
project(ebot_arm_hardware)

# --------------------------------------------------
# Compiler options
# --------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------
# Dependencies
# --------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# --------------------------------------------------
# Build the hardware plugin library
# --------------------------------------------------
add_library(ebot_arm_hardware SHARED
  src/arm_hardware.cpp
)

target_include_directories(ebot_arm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ebot_arm_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

# --------------------------------------------------
# Export plugin description
# --------------------------------------------------
pluginlib_export_plugin_description_file(
  hardware_interface
  ebot_arm_hardware.xml
)

# --------------------------------------------------
# Install rules
# --------------------------------------------------
install(
  TARGETS ebot_arm_hardware
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES ebot_arm_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------
# Testing (optional)
# --------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --------------------------------------------------
# Ament package export
# --------------------------------------------------
ament_package()
