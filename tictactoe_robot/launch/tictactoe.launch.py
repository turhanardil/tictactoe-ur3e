"""
Top-level launch for the tictactoe-ur3e simulation.

Phase 2 scope: spawn the UR3e in our tictactoe world with
joint_state_broadcaster and joint_trajectory_controller active.
MoveIt is launched separately in later phases via ur_sim_moveit.launch.py
or our own move_group launcher.

This file wraps Universal_Robots_ROS2_GZ_Simulation's stock
ur_sim_control.launch.py and only overrides the world file, the
arm series, the initial controller, and RViz visibility.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    world_file = PathJoinSubstitution([
        FindPackageShare("tictactoe_robot"),
        "worlds",
        "tictactoe.sdf",
    ])

    ur_sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ur_simulation_gz"),
            "/launch/ur_sim_control.launch.py",
        ]),
        launch_arguments={
            "ur_type": "ur3e",
            "world_file": world_file,
            "initial_joint_controller": "joint_trajectory_controller",
            "launch_rviz": launch_rviz,
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz in this top-level launch.",
        ),
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Open the Gazebo GUI window.",
        ),
        ur_sim_control,
    ])
