import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import transforms3d

class point_stabilisation_controller(Node):
    def __init__(self):
        super().__init__('point_stabilisation_controller')
        
        # Parámetros del controlador
        self.kp_linear = 1.0
        self.kp_angular = 0.2
        self.max_linear_speed = 5.0
        self.max_angular_speed = 3.0
        self.goal_tolerance = 0.01
        self.angular_tolerance = math.radians(.1)  # 5 grados en radianes
        
        # Estado del robot
        self.current_pose = Point()
        self.current_yaw = 0.0
        self.goal_pose = None
        self.goal_reached = False
        self.orientation_locked = False  # Nuevo estado de orientación
        
        # Subsciptores
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
            
        self.goal_sub = self.create_subscription(
            Point,
            'goal',
            self.goal_callback,
            10)
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        
        # Timer de control
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Control de navegación listo")

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.goal_reached = False
        self.orientation_locked = False  # Resetear estado al nuevo objetivo
        self.get_logger().info(f"Nuevo objetivo recibido: ({msg.x}, {msg.y})")

    def odom_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]

    def control_loop(self):
        if self.goal_pose is None or self.goal_reached:
            return
            
        dx = self.goal_pose.x - self.current_pose.x
        dy = self.goal_pose.y - self.current_pose.y
        distance_error = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        # Máquina de estados
        if not self.orientation_locked:
            # Fase de orientación
            if abs(yaw_error) > self.angular_tolerance:
                self.rotate_to_target(yaw_error)
            else:
                self.orientation_locked = True
                self.get_logger().info("Orientación bloqueada, iniciando movimiento")
        else:
            # Fase de movimiento
            if distance_error < self.goal_tolerance:
                self.finish_goal()
            elif abs(yaw_error) > self.angular_tolerance * 2:  # Tolerancia dinámica
                self.orientation_locked = False
                self.get_logger().info("Reorientando...")
            else:
                self.move_to_target(distance_error)

    def rotate_to_target(self, yaw_error):
        """Control solo para rotación"""
        velocidad_angular = self.kp_angular * yaw_error
        velocidad_angular = max(min(velocidad_angular, self.max_angular_speed), -self.max_angular_speed)
        
        cmd_vel = Twist()
        cmd_vel.angular.z = velocidad_angular
        self.cmd_vel_pub.publish(cmd_vel)

    def move_to_target(self, distance_error):
        """Control solo para movimiento lineal"""
        velocidad_lineal = self.kp_linear * distance_error
        velocidad_lineal = min(velocidad_lineal, self.max_linear_speed)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = velocidad_lineal
        self.cmd_vel_pub.publish(cmd_vel)

    def finish_goal(self):
        self.detener_robot()
        self.goal_reached = True
        self.orientation_locked = False
        self.get_logger().info("¡Objetivo alcanzado!")
        flag_msg = Bool()
        flag_msg.data = True
        self.goal_reached_pub.publish(flag_msg)

    def detener_robot(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    nodo = point_stabilisation_controller()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()