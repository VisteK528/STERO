from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("side_length", description="Length of the square side drawn by TIAGo [m]",
                                        default_value='1.0', ))

    ld.add_action(DeclareLaunchArgument("loops", description="Number of loops that TIAGo will perform",
                                        default_value='1'))

    ld.add_action(
        Node(
            package='rectangle_drawer',
            executable='rectangle_drawer',
            parameters=[
                {"side_length": LaunchConfiguration("side_length"),
                 "loops": LaunchConfiguration("loops")
                 }
            ]
        )
    )

    return ld
