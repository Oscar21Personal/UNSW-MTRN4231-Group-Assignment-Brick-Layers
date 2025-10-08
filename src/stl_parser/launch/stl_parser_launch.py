import launch

from launch_ros.actions import Node
    
def generate_launch_description():

    return launch.LaunchDescription([
        Node(
            package="stl_parser",
            executable="stl_parser",
            name="stl_parser",
            output='screen'
        )
    ])