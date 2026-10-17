import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='rosbot2r').find('rosbot2r')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rosbot.urdf')

    with open(urdf_file, 'r', encoding='utf-8') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_file, '-entity', 'rosbot2r', '-z', '0.2'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_server,
        robot_state_publisher_node,
        spawn_entity
    ])
