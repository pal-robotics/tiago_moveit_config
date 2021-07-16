cmake_minimum_required(VERSION 3.5)
project(tiago_moveit_config)

# PAL Robotics stricter build flags, other developers should not worry about this
find_package(ament_cmake_pal QUIET)
find_package(ament_cmake REQUIRED)

ament_package()


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch.py" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
