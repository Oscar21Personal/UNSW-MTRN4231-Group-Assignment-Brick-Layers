import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/brain_launch.py']
            )
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [get_package_share_directory("stl_parser"), '/launch/stl_parser_launch.py']
            )
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [get_package_share_directory("object_detect"), '/launch/object_detect_launch.py']
            )
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [get_package_share_directory("aruco_detect"), '/launch/aruco_detect_launch.py']
            )
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [get_package_share_directory("motion"), '/launch/motion_launch.py']
            )
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [get_package_share_directory("serial_listener"), '/launch/serial_listener_launch.py']
            )
        ),
    ])