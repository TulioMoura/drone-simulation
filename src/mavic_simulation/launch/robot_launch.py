#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots Mavic 2 Pro driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    # Declare the 'num_drones' launch argument

    package_dir = get_package_share_directory('mavic_simulation')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')

    num_drones = int(input("\n****************************************"
    +"\nHow many drones do you want to simulate?"
    +"\n****************************************\nR:"))

    mavic_drivers = {}
    for i in range(num_drones):
        driver_name = f"mavic_driver_{i + 1}"
        drone_name = f"Mavic_2_PRO_{i+1}"

        mavic_drivers[driver_name] = WebotsController(
            robot_name=drone_name,
            parameters=[
                {'robot_description': robot_description_path},
            ],
            respawn=True
        )
    breakpoint()
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='mavic_world.wbt',
            description='Choose one of the world files from `/mavic_simulation/worlds` directory'
        ),
        webots,
        webots._supervisor,
        # mavic_drivers['mavic_driver_1'],
        # mavic_drivers['mavic_driver_2'],

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])

    for mavic in mavic_drivers:
        ld.add_action(mavic_drivers[mavic])
    # ld.add_action(mavic_drivers['mavic_driver_1'])
    # ld.add_action(mavic_drivers['mavic_driver_2'])

    return ld

    # return LaunchDescription([
    #     DeclareLaunchArgument(
    #         'world',
    #         default_value='mavic_world.wbt',
    #         description='Choose one of the world files from `/mavic_simulation/worlds` directory'
    #     ),
    #     webots,
    #     webots._supervisor,
    #     mavic_drivers['mavic_driver_1'],
    #     mavic_drivers['mavic_driver_2'],

    #     # This action will kill all nodes once the Webots simulation has exited
    #     launch.actions.RegisterEventHandler(
    #         event_handler=launch.event_handlers.OnProcessExit(
    #             target_action=webots,
    #             on_exit=[
    #                 launch.actions.EmitEvent(event=launch.events.Shutdown())
    #             ],
    #         )
    #     )
    # ])