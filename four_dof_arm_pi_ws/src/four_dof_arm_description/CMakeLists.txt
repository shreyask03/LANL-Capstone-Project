cmake_minimum_required(VERSION 3.10)
project(four_dof_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
