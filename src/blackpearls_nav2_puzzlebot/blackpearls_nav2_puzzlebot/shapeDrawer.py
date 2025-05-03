import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class ShapeDrawer(Node):
    def __init__(self):
        super().__init__('shape_drawer')

        # Parámetros de la figura: cuadrado de 3x3 metros
        self.side_length = 3.0
        self.shape_points = [
            (self.side_length, 0.0, 0.0),
            (self.side_length, self.side_length, 0.0),
            (0.0, self.side_length, 0.0),
            (0.0, 0.0, 0.0)
        ]

        # Estado de la secuencia
        self.current_point = 0
        self.waiting_confirmation = False

        # Comunicación ROS 2
        self.goal_pub = self.create_publisher(Point, 'goal', 10)
        self.goal_reached_sub = self.create_subscription(
            Bool,
            'goal_reached',
            self.goal_reached_callback,
            10
        )

        # Temporizador inicial
        self.create_timer(1.0, self.send_first_goal)

        self.get_logger().info("Nodo de dibujo iniciado")

    def send_first_goal(self):
        """Temporizador que envía el primer punto"""
        self.send_next_goal()

    def send_next_goal(self):
        """Publica el siguiente punto del recorrido"""
        if self.waiting_confirmation:
            return

        goal_msg = Point()
        goal_msg.x, goal_msg.y, goal_msg.z = self.shape_points[self.current_point]
        self.goal_pub.publish(goal_msg)

        self.get_logger().info(
            f"Punto {self.current_point + 1}/4 enviado: ({goal_msg.x}, {goal_msg.y})"
        )

        self.current_point = (self.current_point + 1) % len(self.shape_points)
        self.waiting_confirmation = True

    def goal_reached_callback(self, msg):
        """Procesa la llegada a un punto"""
        if msg.data:
            self.get_logger().info("Punto alcanzado, avanzando al siguiente")
            self.waiting_confirmation = False
            self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    drawer = ShapeDrawer()

    try:
        rclpy.spin(drawer)
    except KeyboardInterrupt:
        pass
    finally:
        drawer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
