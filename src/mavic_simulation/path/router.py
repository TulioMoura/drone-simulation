import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def clamp(value, vmin, vmax):
   return min(max(value, vmin), vmax)

class DroneNavigator(Node):
   K_VERTICAL_THRUST = 68.5
   K_VERTICAL_OFFSET = 0.6
   K_VERTICAL_P = 3.0
   K_ROLL_P = 50.0
   K_PITCH_P = 30.0
   MAX_YAW_DISTURBANCE = 0.4
   MAX_PITCH_DISTURBANCE = -1
   target_precision = 0.5


   def __init__(self, cmd_topic: str, waypoints: list, target_altitude: float):
      super().__init__('drone_navigator')
      self.publisher = self.create_publisher(Twist, cmd_topic, 10)
      self.subscriber = self.create_subscription(Odometry, '/drone/pose', self.pose_callback, 10)
      self.timer = self.create_timer(0.05, self.control_loop)
      self.current_pose = [0, 0, 0, 0, 0, 0]
      self.target_position = [0, 0, 0]
      self.target_index = 0
      self.target_altitude = target_altitude
      self.route_concluded = False
      self.waypoints = waypoints
      self.last_time = self.get_clock().now()

   def pose_callback(self, msg: Odometry):
      pos = msg.pose.pose.position
      ori = msg.pose.pose.orientation
      _, _, yaw = self.euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)
      self.current_pose = [pos.x, pos.y, pos.z, 0, 0, yaw]

   @staticmethod
   def euler_from_quaternion(x, y, z, w):
      siny_cosp = 2 * (w * z + x * y)
      cosy_cosp = 1 - 2 * (y * y + z * z)
      yaw = math.atan2(siny_cosp, cosy_cosp)
      return (0, 0, yaw)

   def move_to_target(self):
      if self.target_position[0:2] == [0, 0]:
         self.target_position[0:2] = self.waypoints[0]

      if all(abs(a - b) < self.target_precision for a, b in zip(self.target_position, self.current_pose[0:2])):
         if self.target_index == len(self.waypoints) - 1:
            self.route_concluded = True
            self.target_altitude = 0
         else:
            self.target_index += 1
            self.target_position[0:2] = self.waypoints[self.target_index]

      self.target_position[2] = math.atan2(
         self.target_position[1] - self.current_pose[1],
         self.target_position[0] - self.current_pose[0],
      )
      angle_left = self.target_position[2] - self.current_pose[5]
      angle_left = (angle_left + 2 * math.pi) % (2 * math.pi)
      if angle_left > math.pi:
         angle_left -= 2 * math.pi

      yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * math.pi)
      pitch_disturbance = clamp(math.log10(abs(angle_left) + 1e-3), self.MAX_PITCH_DISTURBANCE, 0.1)
      return yaw_disturbance, pitch_disturbance

   def control_loop(self):
      if self.route_concluded:
         return

      yaw_disturbance, pitch_disturbance = self.move_to_target()

      altitude = self.current_pose[2]
      diff_alt = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
      vertical_input = self.K_VERTICAL_P * pow(diff_alt, 3.0)

      msg = Twist()
      msg.linear.z = vertical_input
      msg.linear.x = pitch_disturbance
      msg.angular.z = yaw_disturbance
      self.publisher.publish(msg)


def run_drone_controller(topic_name: str, waypoints: list, altitude: float):
   rclpy.init()
   node = DroneNavigator(topic_name, waypoints, altitude)
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()


run_drone_controller('/cmd_vel_Drone_043d3943', [[-30, 20], [-60, 20], [-60, 10], [-30, 5]], 10.0)

