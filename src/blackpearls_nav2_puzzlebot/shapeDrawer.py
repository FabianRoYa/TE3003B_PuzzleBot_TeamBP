import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import time

class ShapeDrawer(Node):
    def __init__(self):
        super().__init__('shape_drawer')
        
        # Configuración de la figura (cuadrado de 1x1 metros)
        self.side_length = 3.0  # Lado del cuadrado
        self.shape_points = [
            (self.side_length, 0.0, 0.0),     # Punto inicial
            (self.side_length, self.side_length, 0.0),
            (0.0, self.side_length, 0.0),
            (0.0, 0.0, 0.0)                   # Vuelta al origen
        ]
        
        # Estado del dibujo
        self.current_point = 0
        self.waiting_confirmation = False
        
        # Publicadores y subscriptores
        self.goal_pub = self.create_publisher(Point, 'goal', 10)
        self.goal_reached_sub = self.create_subscription(
            Bool,
            'goal_reached',
            self.goal_reached_callback,
            10)
        
        # Iniciar el proceso después de 1 segundo (para inicialización)
        self.create_timer(1.0, self.send_first_goal)
        
        self.get_logger().info("Dibujante de figuras listo")

    def send_first_goal(self):
        """Envía el primer objetivo al iniciar"""
        self.send_next_goal()

    def goal_reached_callback(self, msg):
        """Maneja la confirmación de punto alcanzado"""
        if msg.data:
            self.get_logger().info("Confirmación recibida, enviando siguiente punto")
            self.waiting_confirmation = False
            self.send_next_goal()

    def send_next_goal(self):
        """Envía el siguiente vértice de la figura"""
        if not self.waiting_confirmation:
            # Preparar mensaje
            goal_msg = Point()
            goal_msg.x, goal_msg.y, goal_msg.z = self.shape_points[self.current_point]
            
            # Publicar y actualizar estado
            self.goal_pub.publish(goal_msg)
            self.get_logger().info(f"Enviado punto {self.current_point + 1}/4: ({goal_msg.x}, {goal_msg.y})")
            
            # Avanzar índice cíclicamente
            self.current_point = (self.current_point + 1) % 4
            self.waiting_confirmation = True

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