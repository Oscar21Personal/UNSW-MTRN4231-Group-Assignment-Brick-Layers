import launch

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
    
def generate_launch_description():

    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return launch.LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(realsense_launch_path),
        #     launch_arguments={
        #         'align_depth.enable': 'true',
        #         'enable_color': 'true',
        #         'enable_depth': 'true'
        #     }.items()
        # ),
        Node(
            package="object_detect",
            executable="object_detect",
            name="object_detect",
            output='screen'
        ),
        Node(
            package="object_detect",
            executable="camera_tf",
            name="camera",
            output='screen'
        ),
        # Node(
        #     package="object_detect",
        #     executable="dummy_free",
        #     name="dummy_free",
        #     output='screen'
        # ),
        # Node(
        #     package="object_detect",
        #     executable="dummy_workspace",
        #     name="dummy_workspace",
        #     output='screen'
        # )
    ])