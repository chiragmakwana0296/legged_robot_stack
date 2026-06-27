cmake_minimum_required(VERSION 3.8)
project(hexapod_msgs)

# Find ament_cmake and rosidl_default_generators
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LegJointsState.msg"
  "msg/LegsJointsState.msg"
  "msg/LegIKRequest.msg"
  "msg/LegPositionState.msg"
  "msg/BodyState.msg"
  "msg/BodyCommand.msg"
  "msg/GaitCommand.msg"
  "srv/GetIKSolver.srv"
  "srv/SetJointPID.srv"
)

ament_package()
