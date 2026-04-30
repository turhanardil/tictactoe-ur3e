# tictactoe-ur3e

Autonomous tic-tac-toe pick-and-place simulation. Universal Robots UR3e on ROS 2 Jazzy + Gazebo Harmonic, picking blue tokens from a tray and placing them on a 3x3 board against a human (mock vision via terminal input).

Duke ECE 383L senior capstone, Spring 2026.

## Stack
- ROS 2 Jazzy on Ubuntu 24.04
- Gazebo Sim Harmonic with Bullet Featherstone physics
- gz_ros2_control + joint_trajectory_controller
- MoveIt 2 (move_group + Pilz industrial motion planner)
- Grasping via Gazebo DetachableJoint plugin (no friction-based grasping)

## Architecture
Three nodes per the proposal, communicating over ROS 2 topics with custom messages.

| Node | Subscribes | Publishes | Purpose |
|------|-----------|-----------|---------|
| `mock_vision` | `/move_command`, `/move_done` | `/board_state` (`tictactoe_msgs/BoardState`) | Terminal input for human moves, tracks full game state |
| `game_ai` | `/board_state` | `/move_command` (`tictactoe_msgs/MoveCommand`) | Minimax decision |
| `arm_control` | `/move_command` | `/move_done` | MoveGroup-action pick-and-place, DetachableJoint attach / detach |

## Setup

The project runs inside the Duke lab Docker container `ros2_lab`. The host directory `~/workspaces/` is bind-mounted to `/root/workspaces/` in the container.

### 1. Create the workspace

```bash
mkdir -p ~/workspaces/tictactoe_ws_v2/src
cd ~/workspaces/tictactoe_ws_v2/src
git clone https://github.com/<your-user>/tictactoe-ur3e.git
```

### 2. Clone source dependencies

The Universal Robots packages are not in the apt sources of this container, so we build them from source. The lab container already has MoveIt 2, ros2_control, ros2_controllers, and gz_ros2_control as `ros-jazzy-*` debs; only the UR repos need cloning.

```bash
cd ~/workspaces/tictactoe_ws_v2
vcs import src < src/tictactoe-ur3e/tictactoe-ur3e.jazzy.repos
```

For a fully fresh ROS 2 Jazzy install (no apt overlay), the same command pulls every dependency.

### 3. Build

Inside the container:

```bash
docker exec -it ros2_lab bash
source /opt/ros/jazzy/setup.bash
cd /root/workspaces/tictactoe_ws_v2
rosdep install --from-paths src --ignore-src -r -y --skip-keys "ur_client_library"
colcon build --packages-up-to ur_simulation_gz --cmake-args -DBUILD_TESTING=OFF
colcon build --packages-select tictactoe_msgs tictactoe_robot --symlink-install
source install/setup.bash
```

`ur_client_library` is skipped because `ur_robot_driver` is real-robot only and not used in simulation. We do not build `ur_robot_driver`, `ur_calibration`, or the `ur` metapackage; `--packages-up-to ur_simulation_gz` excludes them automatically.

### 4. Launch (Phase 2 scope)

```bash
ros2 launch tictactoe_robot tictactoe.launch.py
```

This brings up the UR3e in the tic-tac-toe world with `joint_trajectory_controller` and `joint_state_broadcaster` active. Verify with:

```bash
ros2 control list_controllers
# expected output:
#   joint_trajectory_controller    joint_trajectory_controller/JointTrajectoryController  active
#   joint_state_broadcaster        joint_state_broadcaster/JointStateBroadcaster          active
```

Optional flags:
- `gazebo_gui:=false` to run headless.
- `launch_rviz:=true` to also open RViz.

### 5. Use the game (P3 onward)

Phases P3 through P7 are not yet implemented. See `CLAUDE.md` for the phase plan.

## Project layout

```
tictactoe-ur3e/
  README.md
  CLAUDE.md
  tictactoe-ur3e.jazzy.repos
  tictactoe_msgs/
    package.xml
    CMakeLists.txt
    msg/BoardState.msg
    msg/MoveCommand.msg
  tictactoe_robot/
    package.xml
    setup.py
    setup.cfg
    resource/tictactoe_robot
    launch/tictactoe.launch.py
    worlds/tictactoe.sdf
    config/                         # controllers / moveit configs (P4+)
    tictactoe_robot/
      __init__.py
      scripts/
        __init__.py
        arm_control.py              # P4+
        game_ai.py                  # P3+
        mock_vision.py              # P3+
```

## License
MIT.
