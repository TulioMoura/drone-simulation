

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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import shutil
import os
import json
from launch_ros.actions import Node


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
            drone_content = f"Mavic2Pro {{\n"
            drone_content += f"  translation {drone['path'][0][0]} {drone['path'][0][1]} 0.07\n"  # Adjust translation based on 'i'
            drone_content += "  rotation 0 0 1 3.141590777218456\n"
            drone_content += f"  name \"Mavic_2_PRO_{drone['uuid']}\"\n"
            drone_content += "  controller \"mavic2pro_navigation\" \n"
            #drone_content += "  controller <extern> \n"
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
        
        # Adiciona modelo base do mundo aos drones criados
        wbt_content = template_content + wbt_content
        
        # Verifica se já existe um arquivo updated_world.wbt (se existir dá erro). Se sim, apaga ele.
        if os.path.exists('install/mavic_simulation/share/mavic_simulation/worlds/updated_world.wbt'):
            os.remove('install/mavic_simulation/share/mavic_simulation/worlds/updated_world.wbt')

        # Destino do novo arquivo de mundo que será usado pela simulação
        destination_dir = 'install/mavic_simulation/share/mavic_simulation/worlds/'
        
        # Save the updated WBT content to a new file
        with open('updated_world.wbt', 'w') as output_file:
            output_file.write(wbt_content)

        # Move novo arquivo de mundo para a pasta onde ele será lido posteriormente
        shutil.move('updated_world.wbt', destination_dir)


#method to read the file containing the position for the start of the drone and the 
#path that it needs to follow

def read_path_file(directory):
    #ler o diretorio e retornar um array de objetos    
    print(os.getcwd())
    with open('install/mavic_simulation/share/mavic_simulation/'+ directory, 'r') as drone_path_file:
        drones = json.load(drone_path_file)
    return drones

def generate_launch_description():

    # Método para adicionar a quantidade especificada de drones ao arquivo .wbt do mundo que será simulado
    #path variable is an array of arrays, for each entry, you have an sequence of arrays
    
    # Configurações do mundo que será simulado
    package_dir = get_package_share_directory('mavic_simulation')

    
    robot_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')
    #estrutura de dados que contém as rotas e os drones
    #cada item do array contém um determinado drone

    

    drones = read_path_file('/path/drone_path.json')

    # Obtém a quantidade de drones desejada na simulação
    num_drones = len(drones)


    # Chama o método para adicionar os modelos dos drones no arquivo .wbt do mundo
    generate_wbt_file(drones)

    # Instancia os drivers para a quantidade de drones desejada
    
    #mavic_drivers = {}
    #for i in drones:
    #   driver_name = f"mavic_driver_{i['uuid']}"
    #    drone_name = f"Mavic_2_PRO_{i['uuid']}" 
    #    mavic_drivers[driver_name] = WebotsController(
    #        robot_name=drone_name,
    #        parameters=[
    #            {'robot_description': robot_description_path},
    #        ],
    #        respawn=True
    #)
    
    #webots = WebotsLauncher(
    #    world=os.path.join(package_dir, 'worlds', 'updated_world.wbt')
    #)
    

    # Cria o launch
    ld = LaunchDescription([
        ExecuteProcess(
            cmd=['webots', '--batch', os.path.join(package_dir,'worlds', 'updated_world.wbt' )],
            output='screen',
        ),
        #webots._supervisor,
        # This action will kill all nodes once the Webots simulation has exited
        #launch.actions.RegisterEventHandler(
        #    event_handler=launch.event_handlers.OnProcessExit(
        #        target_action=webots,
        #        on_exit=[
        #            launch.actions.EmitEvent(event=launch.events.Shutdown())
        #        ],
        #    )
        #)
    ])

    #Adiciona os drivers ao launch da simulação
    #for mavic in mavic_drivers:
    #    ld.add_action(mavic_drivers[mavic])
        

    return ld