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
        # Parámetros configurables
        self.declare_parameter('orientation_locked', False)
        self.orientation_locked = self.get_parameter('orientation_locked').get_parameter_value().bool_value

        self.declare_parameter('current_pose', [0.0, 0.0])
        cp = self.get_parameter('current_pose').get_parameter_value().double_array_value
        self.current_pose = Point(x=cp[0], y=cp[1])

        self.declare_parameter('goal_pose', [0.0, 0.0])
        gp = self.get_parameter('goal_pose').get_parameter_value().double_array_value
        self.goal_pose = None if gp == [0.0, 0.0] else Point(x=gp[0], y=gp[1])

        self.declare_parameter('current_yaw', 0.0)
        self.current_yaw = self.get_parameter('current_yaw').get_parameter_value().double_value

        self.declare_parameter('obstacle_detected', False)
        self.obstacle_detected = self.get_parameter('obstacle_detected').get_parameter_value().bool_value

        self.m_line = None  # No es práctico como parámetro

        self.declare_parameter('following_wall', False)
        self.following_wall = self.get_parameter('following_wall').get_parameter_value().bool_value

        self.declare_parameter('safe_distance', 0.5)
        self.safe_distance = self.get_parameter('safe_distance').get_parameter_value().double_value

        self.declare_parameter('safe_wall', 0.45)
        self.safe_wall = self.get_parameter('safe_wall').get_parameter_value().double_value

        self.declare_parameter('angular_tolerance', math.radians(5))
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value

        self.declare_parameter('kp_angular', 0.6)
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value

        self.declare_parameter('kp_linear', 0.2)
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value

        self.declare_parameter('front_distance',float('inf'))
        self.front_distance = self.get_parameter('front_distance').get_parameter_value().double_value

        self.declare_parameter('min_R_wall_distance', float('inf'))
        self.min_R_wall_distance = self.get_parameter('min_R_wall_distance').get_parameter_value().double_value

        self.declare_parameter('min_L_wall_distance', float('inf'))
        self.min_L_wall_distance = self.get_parameter('min_L_wall_distance').get_parameter_value().double_value

        self.declare_parameter('goal_tolerance', 0.5)
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        self.hit_point = None  # No es práctico como parámetro

        self.declare_parameter('goal_reached', False)
        self.goal_reached = self.get_parameter('goal_reached').get_parameter_value().bool_value
   
        self.get_logger().info(f"Inicializando Bug Algorithm en modo: {self.mode}")
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        self.cmd_vel_pub.publish(self.calculate_motion())

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.goal_reached = False
        if self.mode == 'bug2':
            # Guardar posición inicial para m-line
            self.m_line = (Point(x=self.current_pose.x, y=self.current_pose.y), msg)
        self.get_logger().info(f"Nuevo objetivo: ({msg.x}, {msg.y})")

    def odom_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])[2]

    def scan_callback(self, msg):
        front_angles = range(-32,32)
        R_wall_angles = range(-135,-75)
        L_wall_angles = range(75, 135)
        
        # Manejo seguro de listas vacías
        front_readings = [msg.ranges[i] for i in front_angles if 0 < msg.ranges[i] < float('inf')]
        L_wall_readings = [msg.ranges[i] for i in L_wall_angles if 0 < msg.ranges[i] < float('inf')]
        R_wall_readings = [msg.ranges[i] for i in R_wall_angles if 0 < msg.ranges[i] < float('inf')]
        
        self.front_distance = min(front_readings, default=float('inf'))
        self.min_L_wall_distance = min(L_wall_readings, default=float('inf'))
        self.min_R_wall_distance = min(R_wall_readings, default=float('inf'))

        self.obstacle_detected = self.front_distance < self.safe_distance
    def calculate_motion(self):
        if self.goal_pose is None or self.goal_reached:
            return Twist()
            
        dx = self.goal_pose.x - self.current_pose.x
        dy = self.goal_pose.y - self.current_pose.y
        distance = math.hypot(dx, dy)
        
        # Verificación de llegada al objetivo
        if distance <= self.goal_tolerance:
            self.goal_reached = True
            self.get_logger().info("¡¡¡¡¡Goal alcanzado!!!!!")
            return Twist()
        
        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
        
        cmd_vel = Twist()
        
        if not self.orientation_locked:
            if abs(yaw_error) > self.angular_tolerance:
                cmd_vel.angular.z = self.kp_angular * yaw_error
            else:
                self.orientation_locked = True
                self.get_logger().info("Orientación bloqueada, avanzando...")
        else:
            if self.obstacle_detected:
                if self.mode == 'bug0':
                    cmd_vel = self.bug0_behavior()
                elif self.mode == 'bug2':
                    cmd_vel = self.bug2_behavior()
            else:
                cmd_vel.linear.x = 0.1 * distance
        return cmd_vel

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def bug0_behavior(self):
        # Comportamiento original sin cambios
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.01
        
        if self.min_R_wall_distance < self.safe_wall and self.min_L_wall_distance > self.min_R_wall_distance:
            cmd_vel.angular.z = 0.55
        elif self.min_R_wall_distance > self.safe_wall and self.min_L_wall_distance < self.min_R_wall_distance:
            # cmd_vel.linear.x = 0.01
            cmd_vel.angular.z = -0.55
        elif self.min_R_wall_distance < self.safe_wall and self.min_L_wall_distance < self.safe_wall:
            # cmd_vel.linear.x = -0.01
            cmd_vel.angular.z = -0.7
        else:
            # cmd_vel.linear.x = 0.00
            cmd_vel.angular.z = -0.5
        return cmd_vel

    def bug2_behavior(self):
        cmd_vel = Twist()
        if not self.following_wall:
            # Guardar punto de impacto inicial
            self.hit_point = Point(x=self.current_pose.x, y=self.current_pose.y)
            self.following_wall = True
        
        if self.check_m_line_intersection():
            self.following_wall = False
            return self.calculate_motion()
        
        # Seguir la pared (lógica simple)
        cmd_vel.linear.x = 0.1*self.front_distance
        if self.min_R_wall_distance < self.safe_wall and self.min_L_wall_distance > self.min_R_wall_distance:
            cmd_vel.angular.z = 0.7
        elif self.min_R_wall_distance > self.safe_wall and self.min_L_wall_distance < self.min_R_wall_distance:
            cmd_vel.angular.z = -0.5
        elif self.min_R_wall_distance < self.safe_wall and self.min_L_wall_distance < self.safe_wall:
            cmd_vel.angular.z = -0.7
        else:
            cmd_vel.angular.z = -0.5
        return cmd_vel

    def check_m_line_intersection(self):
        if self.mode != 'bug2' or not self.m_line or not self.hit_point:
            return False
        
        # Calcular distancia desde el hit point al objetivo
        hit_to_goal = math.hypot(self.goal_pose.x - self.hit_point.x,
                                 self.goal_pose.y - self.hit_point.y)
        
        # Calcular distancia actual al objetivo
        current_to_goal = math.hypot(self.goal_pose.x - self.current_pose.x,
                                    self.goal_pose.y - self.current_pose.y)
        
        # Verificar si estamos más cerca que el punto de impacto
        return current_to_goal < hit_to_goal and \
            self.point_on_mline(self.current_pose)

    def point_on_mline(self, point, tolerance=0.1):
        # Verificar si el punto está cerca de la m-line
        p1, p2 = self.m_line
        x, y = point.x, point.y
        
        # Ecuación de la recta: (y - y1)(x2 - x1) - (x - x1)(y2 - y1) = 0
        distance = abs((y - p1.y)*(p2.x - p1.x) - (x - p1.x)*(p2.y - p1.y))
        return distance <= tolerance

def main(args=None):
    rclpy.init(args=args)
    node = BugAlgorithm()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()