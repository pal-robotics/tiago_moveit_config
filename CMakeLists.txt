cmake_minimum_required(VERSION 3.5)
project(tiago_moveit_config)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch.py" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
