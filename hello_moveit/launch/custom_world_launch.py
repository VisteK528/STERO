import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from ament_index_python.packages import get_package_share_directory


def list_available_worlds(world_package):
    """
    Funkcja zwraca listę dostępnych plików świata w podanym pakiecie.
    """
    try:
        package_path = get_package_share_directory(world_package)
        worlds_dir = os.path.join(package_path, 'worlds')
        if not os.path.exists(worlds_dir):
            raise FileNotFoundError(f"Nie znaleziono folderu 'worlds' w pakiecie {world_package}")

        # Pobranie plików z rozszerzeniem .world
        available_worlds = [
            f.split(".")[0] for f in os.listdir(worlds_dir) if f.endswith(".world")
        ]
        if not available_worlds:
            raise FileNotFoundError(f"Brak dostępnych plików świata w {worlds_dir}")
        return available_worlds
    except Exception as e:
        raise RuntimeError(f"Błąd przy próbie odczytu dostępnych światów: {e}")


def validate_world_path(world_package, world_name):
    """
    Funkcja walidująca poprawność pakietu i pliku świata.
    """
    try:
        package_path = get_package_share_directory(world_package)
        world_path = os.path.join(package_path, 'worlds', f"{world_name}.world")
        if not os.path.exists(world_path):
            raise FileNotFoundError(f"Plik świata nie istnieje: {world_path}")
        return world_path
    except Exception as e:
        raise RuntimeError(f"Błąd walidacji: {e}")


def generate_launch_description():
    # Deklaracja argumentów
    declare_world_package = DeclareLaunchArgument(
        'world_package',
        default_value='hello_moveit',
        description='Nazwa pakietu, z którego pochodzi plik świata'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='stero',
        description='Nazwa pliku świata bez rozszerzenia .world'
    )

    # Rozwiązanie wartości argumentów (dynamicznie podczas uruchamiania)
    world_package = 'hello_moveit'
    world_name = 'stero'

    try:
        import sys
        for arg in sys.argv:
            if arg.startswith("world_package"):
                world_package = arg.split(":=")[1]
            if arg.startswith("world_name"):
                world_name = arg.split(":=")[1]

        # Wyświetlenie dostępnych światów
        available_worlds = list_available_worlds(world_package)
        print("\nDostępne światy w pakiecie '{}':".format(world_package))
        for i, world in enumerate(available_worlds, start=1):
            print(f"{i}. {world}")
        
        # Interaktywny wybór świata
        world_choice = input("\nWybierz numer świata do uruchomienia: ")
        world_choice = int(world_choice) - 1  # Indeksowanie od 0
        if 0 <= world_choice < len(available_worlds):
            world_name = available_worlds[world_choice]
        else:
            raise ValueError("Niepoprawny wybór świata!")

        print(f"Wybrany świat: {world_name}")

        # Walidacja pliku świata
        resolved_world_path = validate_world_path(world_package, world_name)

        # Komendy Gazebo
        start_gazebo_server = ExecuteProcess(
            cmd=[
                'gzserver', '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so', resolved_world_path
            ],
            output='screen'
        )

        start_gazebo_client = ExecuteProcess(
            cmd=['gzclient'], output='screen'
        )

        return LaunchDescription([
            declare_world_package,
            declare_world_name,
            start_gazebo_server,
            start_gazebo_client
        ])

    except RuntimeError as error:
        # Wyświetlenie komunikatu o błędzie i zakończenie
        return LaunchDescription([
            LogInfo(msg=f"Błąd: {str(error)}"),
        ])
    except ValueError as error:
        # Obsługa niepoprawnego wyboru
        return LaunchDescription([
            LogInfo(msg=f"Błąd: {str(error)}"),
        ])



# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
# from ament_index_python.packages import get_package_share_directory


# def list_available_worlds(world_package):
#     """
#     Funkcja zwraca listę dostępnych plików świata w podanym pakiecie.
#     """
#     try:
#         package_path = get_package_share_directory(world_package)
#         worlds_dir = os.path.join(package_path, 'worlds')
#         if not os.path.exists(worlds_dir):
#             raise FileNotFoundError(f"Nie znaleziono folderu 'worlds' w pakiecie {world_package}")

#         # Pobranie plików z rozszerzeniem .world
#         available_worlds = [
#             f.split(".")[0] for f in os.listdir(worlds_dir) if f.endswith(".world")
#         ]
#         if not available_worlds:
#             raise FileNotFoundError(f"Brak dostępnych plików świata w {worlds_dir}")
#         return available_worlds
#     except Exception as e:
#         raise RuntimeError(f"Błąd przy próbie odczytu dostępnych światów: {e}")


# def validate_world_path(world_package, world_name):
#     """
#     Funkcja walidująca poprawność pakietu i pliku świata.
#     """
#     try:
#         package_path = get_package_share_directory(world_package)
#         world_path = os.path.join(package_path, 'worlds', f"{world_name}.world")
#         if not os.path.exists(world_path):
#             raise FileNotFoundError(f"Plik świata nie istnieje: {world_path}")
#         return world_path
#     except Exception as e:
#         raise RuntimeError(f"Błąd walidacji: {e}")


# def generate_launch_description():
#     # Deklaracja argumentów
#     declare_world_package = DeclareLaunchArgument(
#         'world_package',
#         default_value='hello_moveit',
#         description='Nazwa pakietu, z którego pochodzi plik świata'
#     )

#     declare_world_name = DeclareLaunchArgument(
#         'world_name',
#         default_value='stero',
#         description='Nazwa pliku świata bez rozszerzenia .world'
#     )

#     # Rozwiązanie wartości argumentów (dynamicznie podczas uruchamiania)
#     world_package = 'hello_moveit'
#     world_name = 'stero'

#     try:
#         import sys
#         for arg in sys.argv:
#             if arg.startswith("world_package"):
#                 world_package = arg.split(":=")[1]
#             if arg.startswith("world_name"):
#                 world_name = arg.split(":=")[1]

#         # Wyświetlenie dostępnych światów
#         available_worlds = list_available_worlds(world_package)
#         print("\nDostępne światy w pakiecie '{}':".format(world_package))
#         for i, world in enumerate(available_worlds, start=1):
#             print(f"{i}. {world}")
#         print("\nWybierz świat, używając parametru 'world_name', np.:")
#         print(f"  ros2 launch hello_moveit custom_world_launch.py world_package:={world_package} world_name:={available_worlds[0]}")

#         # Walidacja pliku świata
#         resolved_world_path = validate_world_path(world_package, world_name)

#         # Komendy Gazebo
#         start_gazebo_server = ExecuteProcess(
#             cmd=[
#                 'gzserver', '-s', 'libgazebo_ros_init.so',
#                 '-s', 'libgazebo_ros_factory.so', resolved_world_path
#             ],
#             output='screen'
#         )

#         start_gazebo_client = ExecuteProcess(
#             cmd=['gzclient'], output='screen'
#         )

#         return LaunchDescription([
#             declare_world_package,
#             declare_world_name,
#             start_gazebo_server,
#             start_gazebo_client
#         ])

#     except RuntimeError as error:
#         # Wyświetlenie komunikatu o błędzie i zakończenie
#         return LaunchDescription([
#             LogInfo(msg=f"Błąd: {str(error)}"),
#         ])

# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
# from ament_index_python.packages import get_package_share_directory


# def validate_world_path(world_package, world_name):
#     """
#     Funkcja walidująca poprawność pakietu i pliku świata.
#     """
#     try:
#         package_path = get_package_share_directory(world_package)
#         world_path = os.path.join(package_path, 'worlds', f"{world_name}.world")
#         if not os.path.exists(world_path):
#             raise FileNotFoundError(f"Plik świata nie istnieje: {world_path}")
#         return world_path
#     except Exception as e:
#         raise RuntimeError(f"Błąd walidacji: {e}")


# def generate_launch_description():
#     # Deklaracja argumentów
#     declare_world_package = DeclareLaunchArgument(
#         'world_package',
#         default_value='hello_moveit',
#         description='Nazwa pakietu, z którego pobierany jest plik świata'
#     )

#     declare_world_name = DeclareLaunchArgument(
#         'world_name',
#         default_value='stero',
#         description='Nazwa pliku świata bez rozszerzenia .world'
#     )

#     # Rozwiązanie wartości argumentów (dynamicznie podczas uruchamiania)
#     world_package = 'hello_moveit'
#     world_name = 'stero'

#     try:
#         import sys
#         for arg in sys.argv:
#             if arg.startswith("world_package"):
#                 world_package = arg.split(":=")[1]
#             if arg.startswith("world_name"):
#                 world_name = arg.split(":=")[1]

#         # Walidacja pakietu i pliku świata
#         resolved_world_path = validate_world_path(world_package, world_name)

#         # Komendy Gazebo
#         start_gazebo_server = ExecuteProcess(
#             cmd=[
#                 'gzserver', '-s', 'libgazebo_ros_init.so',
#                 '-s', 'libgazebo_ros_factory.so', resolved_world_path
#             ],
#             output='screen'
#         )

#         start_gazebo_client = ExecuteProcess(
#             cmd=['gzclient'], output='screen'
#         )

#         return LaunchDescription([
#             declare_world_package,
#             declare_world_name,
#             start_gazebo_server,
#             start_gazebo_client
#         ])

#     except RuntimeError as error:
#         # Wyświetlenie komunikatu o błędzie i zakończenie
#         return LaunchDescription([
#             LogInfo(msg=f"Błąd: {str(error)}"),
#         ])





