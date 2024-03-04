cmake_minimum_required(VERSION 3.5)
project(tiago_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(pal_module_cmake QUIET)

if(pal_module_cmake_FOUND)
  pal_register_modules(module/move_group.yaml)
endif()

# Install the moveit setup assistant to avoid some warnings when launching move_group
install(FILES
  .setup_assistant
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
