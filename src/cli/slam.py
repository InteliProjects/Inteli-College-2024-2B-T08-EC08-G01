import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
import math


class NavigationNode(Node):
    def __init__(self):
        super().__init__("robot_navigation")
        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Assina o tópico LaserScan para detectar obstáculos
        self.subscription = self.create_subscription(
            LaserScan, "scan", self.laser_callback, 10
        )

        self.publisher_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.obstacle_detected = False

        # Lista de destinos (x, y, yaw)
        self.destinations = [
            (-1.17, 0.27, 0.00143),
            (-0.5, -0.68, 0.00247),  # Aproximadamente 90 graus
            (-1.0, -2.0, 3.14),  # Aproximadamente 180 graus
        ]
        self.current_goal_index = 0
        self.send_next_goal()

    def move_to_goal(self, index):
        if index < len(self.destinations):
            x, y, yaw = self.destinations[index]
            self.send_goal(x, y, yaw)
        else:
            self.get_logger().info("Todos os destinos foram alcançados.")

    def send_next_goal(self):
        if self.current_goal_index >= len(self.destinations):
            self.current_goal_index = 0  # Reinicia a sequência de destinos
        x, y, yaw = self.destinations[self.current_goal_index]
        self.send_goal(x, y, yaw)

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Define a posição do destino
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Envia a meta de navegação
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info("Goal reached. Moving to the next goal...")
        else:
            self.get_logger().info("Failed to reach goal. Retrying...")

        self.current_goal_index += 1
        self.send_next_goal()

    def laser_callback(self, msg):
        # Lógica para verificar se há um obstáculo próximo
        min_distance = min(msg.ranges)
        if min_distance < 0.5:  # Se o obstáculo estiver a menos de 0.5 metros
            self.obstacle_detected = True
            self.avoid_obstacle()
        else:
            self.obstacle_detected = False

    def avoid_obstacle(self):
        # Comando para desviar do obstáculo
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.5  # Rotaciona o robô para desviar
        self.publisher_cmd_vel.publish(twist_msg)
        self.get_logger().info("Obstacle detected! Avoiding...")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
