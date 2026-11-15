"""Sesame con ros2_control: robot_state_publisher, controller_manager,
controllori delle 4 gambe e RViz.

Uso:
  ros2 launch sesame_bringup control.launch.py                 # hardware finto
  ros2 launch sesame_bringup control.launch.py rviz:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hardware = LaunchConfiguration('hardware')
    rviz = LaunchConfiguration('rviz')

    description_share = FindPackageShare('sesame_description')
    bringup_share = FindPackageShare('sesame_bringup')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([description_share, 'urdf', 'sesame.urdf.xacro']),
            ' hardware:=', hardware,
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([bringup_share, 'config', 'controllers.yaml'])
    rviz_config = PathJoinSubstitution([description_share, 'rviz', 'sesame.rviz'])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # In Jazzy il controller_manager legge l'URDF dal topic /robot_description.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers],
        remappings=[('~/robot_description', '/robot_description')],
    )

    spawners = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            'fr_leg_controller', 'rr_leg_controller',
            'fl_leg_controller', 'rl_leg_controller',
            '--controller-manager', '/controller_manager',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='mock',
                              description='mock | real (real: fase 5)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        robot_state_publisher,
        controller_manager,
        spawners,
        rviz_node,
    ])
