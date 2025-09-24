from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tcp_ip',
            default_value='0.0.0.0',
            description='IP address for TCP endpoint'
        ),
        DeclareLaunchArgument(
            'tcp_port',
            default_value='10000',
            description='Port for TCP endpoint'
        ),
        LogInfo(msg=['Starting Unity TCP Endpoint on port ', LaunchConfiguration('tcp_port')]),
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_endpoint',
            output='screen',
            parameters=[{
                'ROS_IP': LaunchConfiguration('tcp_ip'),
                'ROS_TCP_PORT': LaunchConfiguration('tcp_port'),
            }]
        )
    ])
