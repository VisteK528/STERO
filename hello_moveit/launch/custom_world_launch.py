# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
from os import environ, pathsep

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


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


def start_gzserver(context, *args, **kwargs):
    pkg_path = get_package_share_directory('pal_gazebo_worlds')
    priv_pkg_path = ''
    try:
        priv_pkg_path = get_package_share_directory('hello_moveit')
    except Exception:
        pass

    world_package = LaunchConfiguration('world_package').perform(context)

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

    params_file = PathJoinSubstitution(
        substitutions=[pkg_path, 'config', 'gazebo_params.yaml'])

    # Command to start the gazebo server.
    gazebo_server_cmd_line = [
        'gzserver', '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so', resolved_world_path,
        '--ros-args', '--params-file', params_file]
    # Start the server under the gdb framework.
    debug = LaunchConfiguration('debug').perform(context)
    if debug == 'True':
        gazebo_server_cmd_line = (
                ['xterm', '-e', 'gdb', '-ex', 'run', '--args'] +
                gazebo_server_cmd_line
        )

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=gazebo_server_cmd_line, output='screen')

    return [start_gazebo_server_cmd]


def generate_launch_description():
    # Attempt to find pal_gazebo_worlds_private, use pal_gazebo_worlds otherwise
    try:
        priv_pkg_path = get_package_share_directory(
            'pal_gazebo_worlds_private')
        model_path = os.path.join(priv_pkg_path, 'models') + pathsep
        resource_path = priv_pkg_path + pathsep
    except Exception:
        model_path = ''
        resource_path = ''

    # Add pal_gazebo_worlds path
    pkg_path = get_package_share_directory('pal_gazebo_worlds')
    our_package_path = get_package_share_directory('hello_moveit')

    model_path += os.path.join(pkg_path, 'models')

    resource_path += pkg_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_path += pathsep + environ['GAZEBO_MODEL_PATH']
        model_path += pathsep + os.path.join(our_package_path, 'models')
    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

    declare_world_package = DeclareLaunchArgument(
        'world_package',
        default_value='hello_moveit',
        description='Nazwa pakietu, z którego pochodzi plik świata'
    )

    declare_debug = DeclareLaunchArgument(
        'debug', default_value='False',
        choices=['True', 'False'],
        description='If debug start the gazebo world into a gdb session in an xterm terminal'
    )

    start_gazebo_server_cmd = OpaqueFunction(function=start_gzserver)

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'], output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_debug)
    ld.add_action(declare_world_package)

    ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
    # Using this prevents shared library from being found
    # ld.add_action(SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resource_path))

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld
