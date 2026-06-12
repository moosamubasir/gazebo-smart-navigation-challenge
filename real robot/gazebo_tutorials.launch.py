from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_grid',
            executable='autonomous_robot',
            name='autonomous_robot',
            output='screen'
        ),
        Node(
            package='follow_grid',
            executable='camera_vision',
            name='camera_vision',
            output='screen'
        ),
        Node(
            package='follow_grid',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen'
        ),
        Node(
            package='follow_grid',
            executable='web_dashboard',
            name='web_dashboard',
            output='screen'
        ),
    ])