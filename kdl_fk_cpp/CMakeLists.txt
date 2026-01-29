cmake_minimum_required(VERSION 3.8)
project(kdl_fk_cpp)

# =========================
# Compiler warnings
# =========================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================
# Dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2 REQUIRED)
find_package(trajectory_msgs REQUIRED)


# =========================
# Executables
# =========================

# ---- FK node ----
add_executable(kdl_fk_node
  src/kdl_fk_node.cpp
)

ament_target_dependencies(
  kdl_fk_node
  rclcpp
  sensor_msgs
  urdf
  kdl_parser
  orocos_kdl
  tf2
)

# ---- Cartesian Point Controller ----
add_executable(kdl_cartesian_point_controller
  src/kdl_cartesian_point_controller.cpp
)

ament_target_dependencies(
  kdl_cartesian_point_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs 
  urdf
  kdl_parser
  orocos_kdl
  tf2
)

# =========================
# Install
# =========================
install(
  TARGETS
    kdl_fk_node
    kdl_cartesian_point_controller
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Testing (optional)
# =========================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
