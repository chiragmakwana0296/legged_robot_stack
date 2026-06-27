cmake_minimum_required(VERSION 3.8)
project(hexapod_core)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/hexapod_core/joint_state_to_trajectory.py
  src/hexapod_core/velocity_control.py
  src/hexapod_core/leg_kinematics.py
  src/hexapod_core/kdl_lib.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
