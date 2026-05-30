cmake_minimum_required(VERSION 3.8)
project(bt_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(auv_msgs REQUIRED)

set(DEPS
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  std_msgs
  geometry_msgs
  sensor_msgs
  auv_msgs
)

# --- Shared library of all custom BT nodes ---
add_library(shrub_nodes SHARED
  src/mission_io.cpp
  src/safety_nodes.cpp
  src/nav_nodes.cpp
  src/perception_nodes.cpp
  src/manipulation_nodes.cpp
  src/task_logic_nodes.cpp
)
target_include_directories(shrub_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(shrub_nodes ${DEPS})

# --- Main executor ---
add_executable(bt_executor src/bt_executor.cpp)
target_include_directories(bt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bt_executor shrub_nodes)
ament_target_dependencies(bt_executor ${DEPS})

# --- Install ---
install(TARGETS shrub_nodes ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS bt_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY bt_xml/ DESTINATION share/${PROJECT_NAME}/bt_xml)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY include/ DESTINATION include)

ament_package()
