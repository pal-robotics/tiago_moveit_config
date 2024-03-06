cmake_minimum_required(VERSION 3.5)
project(tiago_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(pal_module_cmake QUIET)

if(pal_module_cmake_FOUND)
  pal_register_modules(module/move_group.yaml)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch.py" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
