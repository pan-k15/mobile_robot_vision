from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    gz_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # RGB image
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',

            # Depth image
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',

            # PointCloud
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_camera_bridge
    ])
