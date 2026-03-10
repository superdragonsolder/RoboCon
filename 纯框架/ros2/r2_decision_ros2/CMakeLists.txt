cmake_minimum_required(VERSION 3.8)
project(r2_decision_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(r2_decision_core_ros2
  ../../cpp/src/state_machine.cpp
  ../../cpp/src/strategy.cpp
)

target_include_directories(r2_decision_core_ros2 PUBLIC
  ../../cpp/include
)

add_executable(launch_manager_node src/launch_manager_node.cpp)
add_executable(mission_state_node src/mission_state_node.cpp)
add_executable(safety_guard_node src/safety_guard_node.cpp)

target_link_libraries(mission_state_node r2_decision_core_ros2)

ament_target_dependencies(launch_manager_node rclcpp std_msgs)
ament_target_dependencies(mission_state_node rclcpp std_msgs)
ament_target_dependencies(safety_guard_node rclcpp std_msgs)

target_link_libraries(launch_manager_node r2_decision_core_ros2)
target_link_libraries(safety_guard_node r2_decision_core_ros2)

install(TARGETS
  launch_manager_node
  mission_state_node
  safety_guard_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
