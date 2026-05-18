cmake_minimum_required(VERSION 3.8)
project(mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(auv_msgs REQUIRED)

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

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  )

add_executable(bt_runner
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/rotate.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/move_to.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/nodes/detect_object.cpp
)

ament_target_dependencies(bt_runner
  rclcpp
  behaviortree_cpp_v3
  std_msgs
  ament_index_cpp
  auv_msgs
)

add_executable(command_adapter
  ${CMAKE_CURRENT_SOURCE_DIR}/command_adapter.cpp
)

ament_target_dependencies(command_adapter
  rclcpp
  std_msgs
)

install(TARGETS
  bt_runner
  command_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../bt_xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()