cmake_minimum_required(VERSION 3.5)
project(union_robot_description)

find_package(ament_cmake REQUIRED)

# Install resource files (URDF, meshes, launch, RViz configs)
install(DIRECTORY
  launch
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
