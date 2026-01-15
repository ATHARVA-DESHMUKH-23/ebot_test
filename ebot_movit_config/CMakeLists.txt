cmake_minimum_required(VERSION 3.22)
project(ebot_movit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    scripts/arm_teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
    scripts/arm_move_action_test.py
  DESTINATION lib/${PROJECT_NAME}
) 