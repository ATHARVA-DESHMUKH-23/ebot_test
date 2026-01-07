cmake_minimum_required(VERSION 3.5)
project(ebot_description)

# Find only what is ACTUALLY required
find_package(ament_cmake REQUIRED)

# Install robot description resources
install(
  DIRECTORY
    launch
    models
    meshes
    config
  DESTINATION
    share/${PROJECT_NAME}
)

# (Optional) install scripts only if you really need them later
# You can even comment this out for now
install(PROGRAMS
  ebot_description/payload.py
  ebot_description/armed_spwaner/spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
