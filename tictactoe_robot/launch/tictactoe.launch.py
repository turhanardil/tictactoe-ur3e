"""
Top-level launch for the tictactoe-ur3e simulation.

Phase 4 scope: arm in tictactoe world, joint_trajectory_controller and
joint_state_broadcaster active, ros_gz_bridge for the 5 attach + 5 detach
topics that drive the DetachableJoint plugins, and MoveIt 2 move_group
running so arm_control.py can plan trajectories.

We wrap ur_simulation_gz/launch/ur_sim_control.launch.py for the Gazebo
spawn and ros2_control side. MoveIt is brought up inline rather than via
ur_simulation_gz/launch/ur_sim_moveit.launch.py because that file does
not forward the world_file argument to ur_sim_control, and because
ur_moveit_config/launch/ur_moveit.launch.py depends on the
wait_for_robot_description executable from ur_robot_driver, which is not
built in our sim-only setup. Replacing it with a TimerAction is enough.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def _bridge_args():
    """One std_msgs/Empty -> gz.msgs.Empty entry per attach and detach topic."""
    args = []
    for i in range(5):
        args.append(
            f'/attach_token_{i}@std_msgs/msg/Empty]gz.msgs.Empty')
        args.append(
            f'/detach_token_{i}@std_msgs/msg/Empty]gz.msgs.Empty')
    return args


def generate_launch_description():
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    ur_type = 'ur3e'

    description_file = PathJoinSubstitution([
        FindPackageShare('tictactoe_robot'), 'urdf', 'tictactoe_arm.urdf.xacro',
    ])
    world_file = PathJoinSubstitution([
        FindPackageShare('tictactoe_robot'), 'worlds', 'tictactoe.sdf',
    ])
    controllers_file = PathJoinSubstitution([
        FindPackageShare('tictactoe_robot'), 'config', 'tictactoe_controllers.yaml',
    ])

    ur_sim_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_simulation_gz'),
            '/launch/ur_sim_control.launch.py',
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'description_file': description_file,
            'world_file': world_file,
            'controllers_file': controllers_file,
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'launch_rviz': 'false',
            'gazebo_gui': gazebo_gui,
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=_bridge_args(),
        output='screen',
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name='ur', package_name='ur_moveit_config')
        .robot_description_semantic(
            Path('srdf') / 'ur.srdf.xacro', {'name': ur_type})
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'publish_robot_description_semantic': True},
        ],
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'), 'config', 'moveit.rviz',
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': True},
        ],
        condition=IfCondition(launch_rviz),
    )

    delayed_moveit = TimerAction(period=8.0, actions=[move_group, rviz])

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='false',
            description='Start RViz with the MoveIt motion planning panel.',
        ),
        DeclareLaunchArgument(
            'gazebo_gui',
            default_value='true',
            description='Open the Gazebo GUI window.',
        ),
        ur_sim_control,
        bridge,
        delayed_moveit,
    ])
