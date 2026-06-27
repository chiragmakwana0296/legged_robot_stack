# legged_robot_stack

Control software stack for legged robots (Hexapod), migrated to **ROS 2 Jazzy**.

---

## Prerequisites

Ensure you have the following installed on your system:
* **ROS 2 Jazzy**
* **xacro**
* **KDL Parser** (`ros-jazzy-kdl-parser` or similar C++ package)
* **PyKDL**

---

## Building the Stack

Build the workspace using `colcon`. If you have local Python environments (such as `pyenv` or custom interpreters in `PATH`), it is recommended to build by filtering the path to ensure CMake compiles ROS 2 Python bindings against the system Python 3.12:

```bash
# Clean previous builds
rm -rf build install log

# Build packages cleanly
PATH=$(echo $PATH | tr ':' '\n' | grep -v "/home/chirag/.local/bin" | grep -v "/home/chirag/.pyenv" | paste -sd:) colcon build
```

---

## Running the Stack

1. **Source the environment** (in every new terminal):
   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ```

2. **Launch the core nodes**:
   This starts the full kinematics pipeline, joint publishers, gait teleop, and control scripts:
   ```bash
   ros2 launch hexapod_core main.launch.py
   ```

3. **Visualize in RViz**:
   Visualize the complete robot model with active joint state transforms:
   ```bash
   ros2 launch leg_description display.launch.py
   ```

---

## Controlling the Robot (Teleop)

You can send commands directly to the robot stack via terminal:

* **Stand Up**:
  ```bash
  ros2 topic pub --once /teleop/body_command hexapod_msgs/msg/BodyCommand '{cmd: 1}'
  ```

* **Sit Down**:
  ```bash
  ros2 topic pub --once /teleop/body_command hexapod_msgs/msg/BodyCommand '{cmd: 2}'
  ```