cmake_minimum_required(VERSION 3.8)
project(hexapod_gait_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(interactive_markers REQUIRED)

include_directories(include)

add_executable(gait_kinematics_node
  src/gait_kinematics.cpp
  src/gait.cpp
)
target_include_directories(gait_kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gait_kinematics_node
  rclcpp
  kdl_parser
  orocos_kdl
  std_msgs
  sensor_msgs
  geometry_msgs
  hexapod_msgs
  interactive_markers
)

add_executable(gait_teleop_node
  src/teleop_joy.cpp
)
target_include_directories(gait_teleop_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gait_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
  hexapod_msgs
)

install(TARGETS
  gait_kinematics_node
  gait_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
