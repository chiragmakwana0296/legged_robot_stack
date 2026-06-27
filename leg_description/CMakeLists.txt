cmake_minimum_required(VERSION 3.8)
project(leg_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
