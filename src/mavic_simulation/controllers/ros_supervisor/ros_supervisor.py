from controller import Supervisor
import rclpy, json, os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class SupervisorNode(Node):
    def __init__(self, supervisor, uuids):
        super().__init__('ros_supervisor')
        self.supervisor = supervisor
        self.timestep = int(supervisor.getBasicTimeStep())
        self.drones = {}
        root = supervisor.getRoot()
        children_field = root.getField("children")
    
        # Lista todos os nós
        #for i in range(children_field.getCount()):
        #    node = children_field.getMFNode(i)
        #    print(node.getTypeName(), node.getDef())

        for uuid in uuids:
            node = supervisor.getFromDef(f"{uuid}")
            if node:
                self.drones[uuid] = {
                    'field': node.getField('translation'),
                    'pub': self.create_publisher(PoseStamped, f'/{uuid}/pose', 10)
                }
            else:
                self.get_logger().warn(f"Drone {uuid} não encontrado no mundo.")

    def step_and_publish(self):
        for uuid, info in self.drones.items():
            pos = info['field'].getSFVec3f()
            msg = PoseStamped()
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.position.z = pos[2]
            info['pub'].publish(msg)

# --- Supervisor principal ---
supervisor = Supervisor()
rclpy.init()

# Lê o mesmo arquivo JSON já usado no projeto
print(os.getcwd())
print(os.listdir('../../path'))
json_path = os.path.join(os.path.dirname(__file__), '../../path/drone_path.json')
with open(json_path) as f:
    drones = json.load(f)
uuids = [d['uuid'] for d in drones]

node = SupervisorNode(supervisor, uuids)

while supervisor.step(node.timestep) != -1:
    node.step_and_publish()
    rclpy.spin_once(node, timeout_sec=0)

