cmake_minimum_required(VERSION 3.8)
project(interfaces)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(livox_to_pointcloud2_node
  src/livox_to_pointcloud2_node.cpp
)
target_compile_features(livox_to_pointcloud2_node PUBLIC cxx_std_17)
ament_target_dependencies(livox_to_pointcloud2_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
)

install(TARGETS livox_to_pointcloud2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
