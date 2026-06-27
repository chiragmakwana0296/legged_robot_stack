cmake_minimum_required(VERSION 3.8)
project(hexapod_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
  src/leg_ik_service.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  kdl_parser
  orocos_kdl
  sensor_msgs
  std_msgs
  geometry_msgs
  hexapod_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
)

add_executable(leg_ik_service_node src/leg_ik_service.cpp)
target_include_directories(leg_ik_service_node PUBLIC include)
ament_target_dependencies(leg_ik_service_node
  rclcpp
  kdl_parser
  orocos_kdl
  sensor_msgs
  std_msgs
  geometry_msgs
  hexapod_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
)

add_executable(leg_kinematics_node src/leg_kinematics.cpp)
ament_target_dependencies(leg_kinematics_node
  rclcpp
  kdl_parser
  orocos_kdl
  sensor_msgs
  std_msgs
  geometry_msgs
  hexapod_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
)

add_executable(hexapod_joint_publisher_node src/joint_publisher.cpp)
ament_target_dependencies(hexapod_joint_publisher_node
  rclcpp
  sensor_msgs
  hexapod_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  leg_ik_service_node
  leg_kinematics_node
  hexapod_joint_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  kdl_parser
  orocos_kdl
  sensor_msgs
  std_msgs
  geometry_msgs
  hexapod_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
)

ament_package()
