

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
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import shutil
import json
import sys



def generate_wbt_file(drones):
	# Abre o modelo base do mundo simulado e lê o conteúdo
        with open('install/mavic_simulation/share/mavic_simulation/worlds/mavic_world.wbt') as template_file:
            template_content = template_file.read()

        # Cria drones de acordo com a quantidade desejada
        wbt_content = ''
        for drone in drones:
            #print(drone)
            #print(f"DRONE-ID {index} INIT X {drone['start'][0]} INIT Y {drone['start'][1]}")
            # Modify template content for each drone instance
            drone_content = f"DEF {drone['uuid']} Mavic2Pro {{\n"
            drone_content += f"  translation {drone['path'][0][0]} {drone['path'][0][1]} 0.3\n"  # Adjust translation based on 'i'
            drone_content += "  rotation 0 0 1 3.141590777218456\n"
            drone_content += f"  name \"{drone['uuid']}\"\n"
            #drone_content += "  controller \"mavic2pro_navigation\" \n"
            drone_content += "  controller \"<extern>\" \n"
            drone_content += f"  customData \"{drone['path']}|{drone['altitude']}\"\n"
            drone_content += "  supervisor TRUE\n"
            drone_content += "  cameraSlot [\n"
            drone_content += "    Camera {\n"
            drone_content += "      width 400\n"
            drone_content += "      height 240\n"
            drone_content += "      near 0.2\n"
            drone_content += "      rotation 0 1 0 1.5708\n"
            drone_content += "    }\n"
            drone_content += "  ]\n"
            drone_content += "}\n\n"

            wbt_content += drone_content

            launch_box = f"CardboardBox {{\n"
            launch_box += f"  name \"cardboardBox_{drone['uuid']}\"\n"
            launch_box += f"translation {drone['path'][0][0]} {drone['path'][0][1]} 0 \n"
            launch_box += "rotation 0 0 1 1.309 \n\n"
            launch_box += "size 2 2 0.2 }"

            wbt_content += launch_box

        
        
        
        # Adiciona modelo base do mundo aos drones criados
        wbt_content = template_content + wbt_content
        
        print("Models Written to world file")
        
        # Verifica se já existe um arquivo updated_world.wbt (se existir dá erro). Se sim, apaga ele.
        dest = 'install/mavic_simulation/share/mavic_simulation/worlds/updated_world.wbt'
        if os.path.exists(dest):
            os.remove(dest)

        with open(dest, 'w') as f:
            f.write(wbt_content)




#method to read the file containing the position for the start of the drone and the 
#path that it needs to follow

def read_path_file(filename):
    with open(os.path.join('install/mavic_simulation/share/mavic_simulation/path', filename), 'r') as f:
        return json.load(f)



def generate_launch_description():
    
    # Método para adicionar a quantidade especificada de drones ao arquivo .wbt do mundo que será simulado
    #path variable is an array of arrays, for each entry, you have an sequence of arrays
    
    # Configurações do mundo que será simulado
    package_dir = get_package_share_directory('mavic_simulation')
    
    robot_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')
    #estrutura de dados que contém as rotas e os drones
    #cada item do array contém um determinado drone


    # lê o argumento file:=filename.json
    path_filename = ""
    for arg in sys.argv:
        if arg.startswith("file:="):
            path_filename = arg.split(":=", 1)[1]

    drones = read_path_file(path_filename)
    # Chama o método para adicionar os modelos dos drones no arquivo .wbt do mundo
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

