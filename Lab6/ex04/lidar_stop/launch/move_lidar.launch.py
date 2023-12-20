import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_stopping = get_package_share_directory('lidar_stop')
    pkg_robot_bringup = get_package_share_directory('robot_bringup')

    lidar_stop = Node(
        package='lidar_stop',
        executable='lidar_stop',
        output='screen',
    )

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_bringup, 'launch', 'diff_drive.launch.py')),
    )

    return LaunchDescription([
        robot_bringup,
        lidar_stop,
    ])
