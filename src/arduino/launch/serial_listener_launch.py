import launch

from launch_ros.actions import Node
    
def generate_launch_description():

    return launch.LaunchDescription([
        Node(
            package="serial_listener",
            executable="serial_listener_node",
            name="serial_listener",
            output='screen'
        )
    ])