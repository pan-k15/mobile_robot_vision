from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_vision')

    rviz_config = os.path.join(pkg_share, 'config', 'robot.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=None,
            output='screen'
        )
    ])
