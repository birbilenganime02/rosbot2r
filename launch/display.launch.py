import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rosbot2r')
    
    urdf_path = os.path.join(pkg_share, 'urdf', 'rosbot.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    if urdf_content.startswith('<?xml'):
        urdf_content = '\n'.join(urdf_content.splitlines()[1:])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': urdf_content}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
