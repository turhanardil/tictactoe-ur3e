# tictactoe-ur3e project conventions

ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2 + UR3e + DetachableJoint plugin.
Two packages: `tictactoe_msgs` (ament_cmake) and `tictactoe_robot` (ament_python).

## Always do
- Run every ROS / colcon / gz command via `docker exec ros2_lab bash -c '...'`.
- Inside that command, source `/opt/ros/jazzy/setup.bash` first, then `/root/workspaces/tictactoe_ws_v2/install/setup.bash`. Do not source `/opt/kortex_ws/install/setup.bash` (it overlays the deprecated Kinova workspace from the prior project and is not needed here).
- Rebuild the affected package after editing: `colcon build --packages-select tictactoe_robot --symlink-install` from `/root/workspaces/tictactoe_ws_v2`.
- For UR or MoveIt sources cloned into `src/`, build the whole tree via `colcon build --packages-up-to ur_simulation_gz --cmake-args -DBUILD_TESTING=OFF`.
- Use `MoveGroup` action via rclpy for arm planning (no MoveItPy in this container). Pilz LIN for Cartesian descents, OMPL for transit. Fall back to a regular pose goal if Pilz LIN fails.
- DetachableJoint is the grasping mechanism. Tokens are owned by independent SDF models. Plugin instances are added to the parent (UR3e) URDF in P4 onward.

## Never do
- Do not use em dashes anywhere. Hyphens, commas, periods, parentheses, "and" are all fine.
- Do not run `ros2`, `colcon`, or `gz` directly on the host. They are not installed there. Always go through `docker exec`.
- Do not pre-solve IK and write static yaml of grasp poses. Compute on the fly via MoveIt IK.
- Do not tune `position_proportional_gain` for `gz_ros2_control` away from the official `ur_simulation_gz` default. The previous Gen3 Lite project tried 0.1 and 5.0; neither helped without other architectural changes.
- Do not add Ruckig smoothing speculatively. None of the working public reference repos use it.
- Do not assume joint-space planning is fine for descents. Use Pilz LIN.
- Do not write more than ~250 lines of `arm_control.py` before testing. Iterate.
- Do not extend the prior `~/workspaces/tictactoe_ws/` repo. It exists for context only.
- Do not skip pre-commit hooks (`--no-verify`). Fix the underlying issue.

## Workspace layout
```
~/workspaces/tictactoe_ws_v2/
  src/
    tictactoe-ur3e/                    # this git repo
      tictactoe_msgs/                  # ament_cmake, BoardState + MoveCommand
      tictactoe_robot/                 # ament_python, world + launch + nodes
      tictactoe-ur3e.jazzy.repos       # source deps (UR + Moveit + ros2_control)
      README.md
      CLAUDE.md                        # this file
    Universal_Robots_ROS2_Description/ # cloned via vcs
    Universal_Robots_ROS2_Driver/      # cloned via vcs
    Universal_Robots_ROS2_GZ_Simulation/  # cloned via vcs
    ur_msgs/                           # cloned via vcs
```

## Phase order (build incrementally, stop after each phase)
P1. `tictactoe_msgs` package, BoardState + MoveCommand interfaces installable.
P2. World SDF + UR3e spawn via wrapped `ur_sim_control.launch.py`. JTC + JSB active.
P3. `mock_vision` and `game_ai` Python nodes, terminal-only, no arm motion.
P4. `arm_control.py` single-token sanity. Pick token 0, place at cell 4. DetachableJoint added in P4.
P5. Generalize over all 5 tray slots and 9 board cells.
P6. Visual gripper for the demo. Red token spawning when human plays.
P7. Full integration test, multiple games.

## Topics and messages
- `/board_state` (`tictactoe_msgs/BoardState`, 9 ints, 0/1/2). Published by mock_vision.
- `/move_command` (`tictactoe_msgs/MoveCommand`, single int 0-8). Published by game_ai.
- `/move_done` (`std_msgs/Bool`). Published by arm_control after a move executes.
- `/attach_token_<n>`, `/detach_token_<n>` (`std_msgs/Empty`). ROS to gz bridge for DetachableJoint, one pair per blue token.

## Smoke checks per phase
- After build: `docker exec ros2_lab bash -c 'source ... && ros2 pkg list | grep tictactoe'` shows both packages.
- After P2 launch: `ros2 control list_controllers` returns `joint_trajectory_controller active` and `joint_state_broadcaster active`. `/joint_states` publishes 6 UR joint values.
- After P4: published joint trajectory completes without an end-of-trajectory velocity spike, token visibly attaches and detaches.
