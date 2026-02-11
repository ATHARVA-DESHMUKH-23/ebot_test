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
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ebot_navigation_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)


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

add_executable(pick_fsm_node src/pick_fsm_node.cpp)

ament_target_dependencies(pick_fsm_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  ebot_navigation_interfaces
  visualization_msgs
  control_msgs
  trajectory_msgs
)

add_executable(kdl_cartesian_servo src/kdl_cartesian_servo.cpp)

ament_target_dependencies(kdl_cartesian_servo
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs 
  urdf
  kdl_parser
  orocos_kdl
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(kdl_singularity_map src/kdl_singularity_map.cpp)

ament_target_dependencies(kdl_singularity_map
  rclcpp
  visualization_msgs
  urdf
  kdl_parser
  orocos_kdl
)

add_executable(point_integration src/point_integration.cpp)

ament_target_dependencies(point_integration
  rclcpp
  geometry_msgs
  visualization_msgs
)

add_executable(kdl_workspace_cloud src/kdl_workspace_cloud.cpp)

ament_target_dependencies(kdl_workspace_cloud
  rclcpp
  visualization_msgs
  urdf
  kdl_parser
  orocos_kdl
)

add_executable(kdl_entire_body_servoing src/kdl_entire_body_servoing.cpp)

ament_target_dependencies(kdl_entire_body_servoing
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  ebot_navigation_interfaces
  visualization_msgs
  control_msgs
  trajectory_msgs
  urdf
  kdl_parser
  orocos_kdl
)

# =========================
# Install
# =========================
install(
  TARGETS
    kdl_fk_node
    kdl_cartesian_point_controller
    pick_fsm_node
    kdl_cartesian_servo
    kdl_singularity_map
    point_integration
    kdl_workspace_cloud
    kdl_entire_body_servoing
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
