cmake_minimum_required(VERSION 3.8)
project(dog_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(dog_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(BPX_SDK_ROOT "$ENV{BPX_SDK_ROOT}" CACHE PATH "Path to bpx_sdk_open")
if(NOT BPX_SDK_ROOT)
  set(BPX_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../3rd_utils/bpx_sdk_open")
endif()

find_path(BPX_SDK_INCLUDE_DIR
  NAMES request_robot_state.h
  HINTS "${BPX_SDK_ROOT}/include"
  REQUIRED
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(BPX_SDK_LIBRARY_NAME bpx_sdk_aarch64)
else()
  set(BPX_SDK_LIBRARY_NAME bpx_sdk_x86_64)
endif()

find_library(BPX_SDK_LIBRARY
  NAMES ${BPX_SDK_LIBRARY_NAME}
  HINTS "${BPX_SDK_ROOT}/lib"
  REQUIRED
)

add_library(bpx_sdk SHARED IMPORTED)
set_target_properties(bpx_sdk PROPERTIES
  IMPORTED_LOCATION "${BPX_SDK_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${BPX_SDK_INCLUDE_DIR}"
)

add_executable(dog_driver_node
  src/bpx_read_only_client.cpp
  src/dog_driver_node.cpp
  src/main.cpp
)

target_include_directories(dog_driver_node PRIVATE include)
target_link_libraries(dog_driver_node bpx_sdk)
ament_target_dependencies(dog_driver_node
  diagnostic_msgs
  dog_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
)

get_filename_component(BPX_SDK_LIBRARY_DIR "${BPX_SDK_LIBRARY}" DIRECTORY)
set_target_properties(dog_driver_node PROPERTIES
  BUILD_RPATH "${BPX_SDK_LIBRARY_DIR}"
  INSTALL_RPATH "${BPX_SDK_LIBRARY_DIR}"
)

install(TARGETS dog_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
