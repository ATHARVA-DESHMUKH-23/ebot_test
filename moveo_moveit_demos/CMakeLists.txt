cmake_minimum_required(VERSION 3.8)
project(moveo_moveit_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)
install(TARGETS hello_moveit
  DESTINATION lib/${PROJECT_NAME})

  add_executable(hello_moveit_full_body src/hello_moveit_full_body.cpp)
  target_include_directories(hello_moveit_full_body PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_compile_features(hello_moveit_full_body PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(
    hello_moveit_full_body
    "moveit_ros_planning_interface"
    "rclcpp"
  )
  install(TARGETS hello_moveit_full_body
    DESTINATION lib/${PROJECT_NAME})

add_executable(moveit_aruco src/moveit_aruco.cpp)

target_include_directories(moveit_aruco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(moveit_aruco PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  moveit_aruco
  "moveit_ros_planning_interface"
  "rclcpp"
)

add_executable(hello_moveit_pose src/hello_moveit_pose.cpp)

target_include_directories(hello_moveit_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(hello_moveit_pose PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  hello_moveit_pose
  "moveit_ros_planning_interface"
  "rclcpp"
)


install(TARGETS moveit_aruco
  DESTINATION lib/${PROJECT_NAME})

add_executable(plan_around_objects src/plan_around_objects.cpp)
target_include_directories(plan_around_objects PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(plan_around_objects PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  plan_around_objects
  "moveit_msgs"
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)
install(TARGETS plan_around_objects
  DESTINATION lib/${PROJECT_NAME})

add_executable(mobile_manipulator_planner src/mobile_manipulator_planner.cpp)

ament_target_dependencies(mobile_manipulator_planner
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

add_executable(workspace_scan_node src/workspace_scan_node.cpp)

target_include_directories(workspace_scan_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(workspace_scan_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  workspace_scan_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(TARGETS workspace_scan_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS mobile_manipulator_planner
  DESTINATION lib/${PROJECT_NAME})


  
add_executable(arm_demo_loop src/arm_demo_loop.cpp)

target_include_directories(arm_demo_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(arm_demo_loop PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  arm_demo_loop
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS arm_demo_loop
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(moma_demo_loop src/moma_demo_loop.cpp)

target_include_directories(moma_demo_loop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(moma_demo_loop PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  moma_demo_loop
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  moveit_ros_planning_interface
)

install(TARGETS moma_demo_loop
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(target_approach_frame src/target_approach_frame.cpp)

ament_target_dependencies(target_approach_frame
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_msgs
)

install(TARGETS target_approach_frame
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(smart_target_approach src/smart_target_approach.cpp)

target_compile_features(smart_target_approach PUBLIC cxx_std_17)

ament_target_dependencies(smart_target_approach
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_msgs
)

install(TARGETS smart_target_approach
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(moveit_goal src/moveit_goal.cpp)

target_include_directories(moveit_goal PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(moveit_goal PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  moveit_goal
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS moveit_goal
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(fake_servo src/fake_servo.cpp)

target_include_directories(fake_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(fake_servo PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  fake_servo
  rclcpp
  geometry_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2
  tf2_geometry_msgs
)

install(TARGETS fake_servo
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
