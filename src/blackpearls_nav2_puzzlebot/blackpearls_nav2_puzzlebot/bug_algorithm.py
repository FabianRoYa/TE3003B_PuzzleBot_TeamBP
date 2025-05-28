# bug_algorithm.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import transforms3d

class BugAlgorithm(Node):
    def __init__(self):
        super().__init__('bug_algorithm')
        
        # Parámetros del algoritmo
        self.declare_parameter('mode', 'bug0')  # 'bug0' o 'bug2'
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        
        # Suscriptores
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'ground_truth', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Point, 'goal', self.goal_callback, 10)
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Variables de estado
        self.orientation_locked = False  # Indica si el robot está orientado hacia el objetivo
        self.current_pose = Point()
        self.goal_pose = None
        self.current_yaw = 0.0  # Orientación actual del robot en radianes
        self.obstacle_detected = False
        self.R_wall = False
        self.m_line = None  # Solo para Bug2
        self.following_wall = False
        self.safe_distance = 0.3  # Distancia mínima a obstáculos [m]
        self.safe_wall = 0.4
        self.angular_tolerance = math.radians(5)  # Tolerancia angular [rad]
        self.kp_angular = 0.1
        self.kp_linear = 0.2

        # timers
        self.create_timer(0.1, self.timer_callback)  # Control loop cada 100 ms
    
    def timer_callback(self):
        self.cmd_vel_pub.publish(self.calculate_motion())

    def goal_callback(self, msg):
        self.goal_pose = msg
        if self.mode == 'bug2':
            self.m_line = (self.current_pose, self.goal_pose)  # Definir m-line
        self.get_logger().info(f"Nuevo objetivo: ({msg.x}, {msg.y})")

    def odom_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]

    def scan_callback(self, msg):
        # Detectar obstáculos en un sector frontal
        front_angles = range(-30, 30)  # Rango de 60 grados
        wall_angles = range(-100,-80)  # pared derecha
        min_distance = min([msg.ranges[i] for i in front_angles if not math.isinf(msg.ranges[i])])
        self.obstacle_detected = min_distance < self.safe_distance
        min_wall_distance = min([msg.ranges[i] for i in wall_angles if not math.isinf(msg.ranges[i])])
        self.R_wall = min_wall_distance < self.safe_wall

        # Imprimir las distancias mínimas detectadas
        self.get_logger().info(f"Distancia mínima al frente: {min_distance:.2f} m, pared derecha: {min_wall_distance:.2f} m")

    def calculate_motion(self):
        if self.goal_pose is None:
            return Twist()
            
        dx = self.goal_pose.x - self.current_pose.x
        dy = self.goal_pose.y - self.current_pose.y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
        
        cmd_vel = Twist()
        
        # Máquina de estados
        if not self.orientation_locked:
            # Fase 1: Orientación hacia el objetivo
            if abs(yaw_error) > self.angular_tolerance:
                cmd_vel.angular.z = self.kp_angular * yaw_error
            else:
                self.orientation_locked = True
                self.get_logger().info("Orientación bloqueada, avanzando...")
        else:
            # Fase 2: Movimiento hacia el objetivo
            if self.obstacle_detected:
                if self.mode == 'bug0':
                    cmd_vel = self.bug0_behavior()
                elif self.mode == 'bug2':
                    cmd_vel = self.bug2_behavior()  # Lógica Bug0/Bug2
            else:
                cmd_vel.linear.x = 0.1 * distance
                # Pequeñas correcciones angulares mientras avanza
                cmd_vel.angular.z = 0.1 * yaw_error
                
            
        
        return cmd_vel

    def normalize_angle(self, angle):
        # Normalizar el ángulo a [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))

    def bug0_behavior(self):
        # Comportamiento simple de seguimiento de pared para Bug0
        cmd_vel = Twist()
        # Gira a la izquierda para evitar el obstáculo y sigue la pared
        cmd_vel.linear.x = 0.01
        if self.R_wall:
            cmd_vel.angular.z = 0.3
        else:
            cmd_vel.angular.z = -0.3
        return cmd_vel

    def bug2_behavior(self):
        # Seguir el obstáculo hasta intersectar la m-line
        cmd_vel = Twist()
        if self.check_m_line_intersection():
            self.following_wall = False
            return self.calculate_motion()
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = -0.5
            return cmd_vel

    def check_m_line_intersection(self):
        # Verificar si la posición actual está cerca de la m-line (Bug2)
        pass  # Implementar lógica de intersección

def main(args=None):
    rclpy.init(args=args)
    node = BugAlgorithm()  # Cambiar a 'bug0' o 'bug2' según sea necesario
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()