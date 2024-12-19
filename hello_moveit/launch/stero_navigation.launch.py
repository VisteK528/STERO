import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Deklaracja argumentów
    declare_world_package = DeclareLaunchArgument(
        'world_package',
        default_value='hello_moveit',
        description='Nazwa pakietu, z którego pochodzi plik świata'
    )

    declare_navigation = DeclareLaunchArgument(
        'navigation',
        default_value='True',
        description='Czy włączyć nawigację'
    )

    declare_moveit = DeclareLaunchArgument(
        'moveit',
        default_value='True',
        description='Czy włączyć MoveIt'
    )

    declare_is_public_sim = DeclareLaunchArgument(
        'is_public_sim',
        default_value='True',
        description='Czy uruchomiona symulacja jest publiczna'
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Czy włączyć SLAM'
    )

    # Pobranie ścieżki do launch z kroku 2(b) (custom_world_launch.py)
    custom_launch_path = os.path.join(
        get_package_share_directory('hello_moveit'),
        'launch',
        'custom_world_launch.py'
    )

    # Włączenie pliku launch z kroku 2(b)
    include_custom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_launch_path),
        launch_arguments={
            'world_package': LaunchConfiguration('world_package'),
        }.items()
    )

    # Dodanie węzła SLAM, jeśli 'slam' jest ustawione na True
    slam_node = None
    if LaunchConfiguration('slam') == 'True':
        slam_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch',
                    'slam_toolbox_launch.py'  # Zmienna nazwa zależna od używanego SLAM
                )
            ),
            launch_arguments={}.items()
        )

    # Tworzenie opisu launch
    actions = [
        declare_world_package,
        declare_world_name,
        declare_navigation,
        declare_moveit,
        declare_is_public_sim,
        declare_slam,
        include_custom_launch
    ]

    if slam_node:
        actions.append(slam_node)

    return LaunchDescription(actions)

