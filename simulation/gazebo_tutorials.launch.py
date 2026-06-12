import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_tutorial')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(pkg_share, 'worlds', 'bonus_grid.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items(),
    )

    controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='gazebo_tutorial',
                executable='bonus_randomizer',
                name='bonus_randomizer',
                output='screen',
            ),
        ],
    )

    collector = TimerAction(
        period=9.0,
        actions=[
            Node(
                package='gazebo_tutorial',
                executable='collector_controller',
                name='collector_controller',
                output='screen',
            )
        ],
    )

    return LaunchDescription([gazebo, controller, collector])
