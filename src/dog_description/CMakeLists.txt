cmake_minimum_required(VERSION 3.8)
project(dog_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch simply_mesh urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "mesh_report.json" EXCLUDE
)

install(
  FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
