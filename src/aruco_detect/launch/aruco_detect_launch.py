import launch

from launch_ros.actions import Node
    
def generate_launch_description():

    return launch.LaunchDescription([
        Node(
            package="aruco_detect",
            executable="aruco_detect",
            name="aruco_detect",
            output='screen'
        ),
        Node(
            package="aruco_detect",
            executable="board_publish",
            name="board_publish",
            output='screen'
        )
    ])