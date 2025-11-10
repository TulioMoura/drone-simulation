import os
import sys
import json
import shutil
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def read_path_file(filename):
    with open(os.path.join('install/mavic_simulation/share/mavic_simulation/path', filename), 'r') as f:
        return json.load(f)


def generate_wbt_file(drones):
    template_path = 'install/mavic_simulation/share/mavic_simulation/worlds/mavic_world.wbt'
    with open(template_path) as f:
        template_content = f.read()

    wbt_content = template_content

    for drone in drones:
        name = drone['uuid']
        x, y = drone['path'][0][0], drone['path'][0][1]
        altitude = drone.get('altitude', 0.3)

        drone_block = f"""
DEF {name} Mavic2Pro {{
  translation {x} {y} {altitude}
  rotation 0 0 1 3.14159
  name "{name}"
  controller "<extern>"
  customData "{drone['path']}|{altitude}"
}}
"""
        wbt_content += drone_block

    supervisor_block = """
Robot {
  name "ros_supervisor"
  supervisor TRUE
  controller "ros_supervisor"
}
"""
    wbt_content += supervisor_block

    dest = 'install/mavic_simulation/share/mavic_simulation/worlds/updated_world.wbt'
    if os.path.exists(dest):
        os.remove(dest)

    with open(dest, 'w') as f:
        f.write(wbt_content)


def generate_launch_description():
    package_dir = get_package_share_directory('mavic_simulation')
    robot_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')

    # lê o argumento file:=filename.json
    path_filename = ""
    for arg in sys.argv:
        if arg.startswith("file:="):
            path_filename = arg.split(":=", 1)[1]

    drones = read_path_file(path_filename)
    generate_wbt_file(drones)

    world_path = os.path.join(package_dir, 'worlds', 'updated_world.wbt')
    webots = WebotsLauncher(world=world_path)

    ld = LaunchDescription([
        SetEnvironmentVariable(name='file', value=path_filename),
        webots,  # executa Webots
    ])

    for drone in drones:
        controller = WebotsController(
            robot_name=drone['uuid'],
            parameters=[{'robot_description': robot_description_path}],
            respawn=True,
            output='screen'
        )
        ld.add_action(controller)

    # encerra simulação ao fechar o Webots
    from launch.event_handlers import OnProcessExit
    from launch.actions import RegisterEventHandler, EmitEvent
    from launch.events import Shutdown

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    )

    return ld

