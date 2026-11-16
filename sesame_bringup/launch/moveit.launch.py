"""sesame_bringup/launch/moveit.launch.py

Fase 2: ros2_control (i nostri 4 controllori) + move_group + RViz con MotionPlanning.
Il robot_description viene dal NOSTRO xacro, così c'è una sola fonte di verità.

Uso:
  ros2 launch sesame_bringup moveit.launch.py                  # hardware finto
  ros2 launch sesame_bringup moveit.launch.py hardware:=real   # ESP32 (fase 5)

L'argomento hardware arriva a control.launch.py, cioè al controller_manager.
move_group e RViz usano l'URDF generato con hardware:=mock: il blocco
ros2_control non conta per MoveIt, giunti e limiti sono gli stessi.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_share = get_package_share_directory("sesame_description")
    bringup_share = get_package_share_directory("sesame_bringup")
    moveit_share = get_package_share_directory("sesame_moveit_config")

    xacro_file = os.path.join(description_share, "urdf", "sesame.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("sesame_quadruped", package_name="sesame_moveit_config")
        .robot_description(file_path=xacro_file, mappings={"hardware": "mock"})
        .robot_description_semantic(file_path="config/sesame_quadruped.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Fase 1 così com'è, ma senza il suo RViz (usiamo quello di MoveIt)
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "control.launch.py")),
        launch_arguments={"rviz": "false", "hardware": LaunchConfiguration("hardware")}.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("hardware", default_value="mock",
                              description="mock | real (real: ESP32 con micro-ROS, fase 5)"),
        control,
        move_group,
        rviz,
    ])
