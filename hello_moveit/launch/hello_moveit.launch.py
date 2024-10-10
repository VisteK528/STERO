from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="hello_moveit",
            executable="hello_moveit",
            parameters=[{"use_sim_time": True}]
        )
    )

    return ld
