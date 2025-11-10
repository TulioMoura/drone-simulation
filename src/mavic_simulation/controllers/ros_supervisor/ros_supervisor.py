from controller import Supervisor
import rclpy, json, os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class SupervisorNode(Node):
    def __init__(self, supervisor, uuids):
        super().__init__('ros_supervisor')
        self.supervisor = supervisor
        self.timestep = int(supervisor.getBasicTimeStep())
        self.drones = {}

        for uuid in uuids:
            node = supervisor.getFromDef(uuid)
            if node:
                self.drones[uuid] = {
                    'node': node,
                    'field': node.getField('translation'),
                    'pub_pose': self.create_publisher(PoseStamped, f'/{uuid}/pose', 10),
                    'pub_custom': self.create_publisher(String, f'/{uuid}/custom_data', 10),
                }
                self.get_logger().info(f"Drone {uuid} registrado.")
            else:
                self.get_logger().warn(f"Drone {uuid} não encontrado no mundo.")

    def step_and_publish(self):
        for uuid, info in self.drones.items():
            # posição
            pos = info['field'].getSFVec3f()
            pose_msg = PoseStamped()
            pose_msg.pose.position.x = pos[0]
            pose_msg.pose.position.y = pos[1]
            pose_msg.pose.position.z = pos[2]
            info['pub_pose'].publish(pose_msg)

            # customData
            custom = info['node'].getCustomData() or ''
            msg = String()
            msg.data = custom
            info['pub_custom'].publish(msg)


def main():
    supervisor = Supervisor()
    rclpy.init()

    filename = os.environ.get("file")
    if not filename:
        raise RuntimeError("Variável de ambiente 'file' não definida.")

    json_path = os.path.join(
        os.path.dirname(__file__),
        '../../path/',
        filename
    )

    with open(json_path) as f:
        drones = json.load(f)
    uuids = [d['uuid'] for d in drones]

    node = SupervisorNode(supervisor, uuids)

    while supervisor.step(node.timestep) != -1 and rclpy.ok():
        node.step_and_publish()
        rclpy.spin_once(node, timeout_sec=0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
