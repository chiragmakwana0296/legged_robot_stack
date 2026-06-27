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

### 1. Using Standard `/cmd_vel` (Twist Interface)

We added a new package `hexapod_teleop` that provides a standard `geometry_msgs/msg/Twist` (`/cmd_vel`) subscriber. This enables controlling the robot using generic tools like `teleop_twist_keyboard` or `rqt_robot_steering`.

* **Launch the Teleop Twist Node**:
  ```bash
  ros2 launch hexapod_teleop teleop.launch.py
  ```

* **Control via Keyboard**:
  Run standard ROS 2 keyboard teleop:
  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```

* **Control via Terminal Command**:
  Send a linear forward velocity of 0.1 m/s:
  ```bash
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```

* **Stand Up / Sit Down Control**:
  You can manually trigger standing up or sitting down via the `/teleop/stand` boolean topic:
  - **Stand Up**:
    ```bash
    ros2 topic pub --once /teleop/stand std_msgs/msg/Bool "{data: true}"
    ```
  - **Sit Down**:
    ```bash
    ros2 topic pub --once /teleop/stand std_msgs/msg/Bool "{data: false}"
    ```

> [!NOTE]
> The teleop node has an `auto_stand_up` feature (enabled by default) that automatically stands the robot up when a non-zero velocity command is first received. It also includes a watchdog timeout (default 0.5s) to pause the gait if `/cmd_vel` commands stop publishing.

---

### 2. Direct Body Commands

You can also send direct commands to the body kinematics node:

* **Stand Up**:
  ```bash
  ros2 topic pub --once /teleop/body_command hexapod_msgs/msg/BodyCommand '{cmd: 1}'
  ```

* **Sit Down**:
  ```bash
  ros2 topic pub --once /teleop/body_command hexapod_msgs/msg/BodyCommand '{cmd: 2}'
  ```