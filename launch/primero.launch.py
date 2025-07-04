from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('cuadrupedo'),
        'urdf',
        'LeggedRobot.urdf'
    )
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    rviz_config = os.path.join(
        get_package_share_directory('cuadrupedo'),
        'rviz',
        'cuadrupedoRviz.rviz'
    )

    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        ),
        Node(
            package='cuadrupedo',
            executable='caminar',
            name='caminar'
        ),
    ])
