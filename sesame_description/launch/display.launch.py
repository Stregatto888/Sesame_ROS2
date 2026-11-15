from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():

    # Individua la directory installata del pacchetto
    package_dir = Path(get_package_share_directory('sesame_description'))

    # Percorsi del modello URDF e della configurazione di RViz
    urdf_file = package_dir / 'urdf' / 'sesame.urdf'
    rviz_config = package_dir / 'rviz' / 'sesame.rviz'

    # Legge il contenuto dell'URDF
    robot_description = urdf_file.read_text()

    # Pubblica lo stato e le trasformazioni del robot
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # Interfaccia grafica per controllare i giunti
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # Visualizzatore 3D con la configurazione salvata
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config)]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
