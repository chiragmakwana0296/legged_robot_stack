cmake_minimum_required(VERSION 3.8)
project(hexapod_body_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}_node
  src/body_kinematics.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  kdl_parser
  orocos_kdl
  std_msgs
  hexapod_msgs
  interactive_markers
  geometry_msgs
)

add_executable(interactive_marker_node
  src/interactive_marker.cpp
)
target_include_directories(interactive_marker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(interactive_marker_node
  rclcpp
  interactive_markers
)

install(TARGETS
  ${PROJECT_NAME}_node
  interactive_marker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
