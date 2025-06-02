import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np

class ControllerClass(Node):
    def __init__(self):
        super().__init__('point_stabilisation_controller')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.initial_x = self.get_parameter('x').value
        self.initial_y = self.get_parameter('y').value
        timer_period = 0.1
        self.goal_x = self.initial_x
        self.goal_y = self.initial_y
        self.x = self.initial_x
        self.y = self.initial_y
        self.theta = 0.0
        self.Kp_linear = 0.1
        self.Kp_angular = 0.3
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.3
        self.follow_distance = 0.8
        self.stop_d = 0.05
        self.turning_d = np.deg2rad(5)
        self.follow = False
        self.lidar_recieved = False
        self.begin_follow = False
        self.lidar = LaserScan()
        self.robot_vel = Twist()
        self.next_point = Bool()
        self.inf1 = String()
        self.inf2 = String()
        #self.tf_br2 = TransformBroadcaster(self)
        #self.t = TransformStamped()
        self.ll = []
        self.zero = (np.zeros(360, dtype=np.float32) + 0.05).tolist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sub_p = self.create_subscription(Odometry, 'ground_truth', self.pose_callback, 10)
        self.sub_g = self.create_subscription(Point, 'goal', self.goal_callback, 10)
        self.sub_l = self.create_subscription(LaserScan, "scan", self.lidar_callback, 10)
        self.pub_v = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(String, 'inf1', 10)
        self.pub2 = self.create_publisher(String, 'inf2', 10)

        self.start_time = self.get_clock().now()

    def get_view(self, r, center, limit, max_distance):
        self.ll = []
        detect = []
        i_i = center - limit
        i_f = center + limit
        if i_i < 0:
            for i in range(0, i_f):
                self.ll.append(r[i])
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
            for i in range(360 - i_i, 360):
                self.ll.append(r[i])
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
        elif i_f > 360:
            for i in range(i_i, 360):
                self.ll.append(r[i])
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
            for i in range(0, i_f - 360):
                self.ll.append(r[i])
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
        else:
            for i in range(i_i, i_f):
                self.ll.append(r[i])
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
        if np.sum(detect) == 0:
            view = "clear"
        else:
            view = "obstacle"
        return view

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = transforms3d.euler.quat2euler([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])

    def goal_callback(self, msg1):
        self.goal_x = msg1.x
        self.goal_y = msg1.y

    def lidar_callback(self, msg2):
        self.lidar = msg2
        self.lidar_recieved = True

    def timer_callback(self):
        if self.lidar_recieved:
            goal_reached = False
            stopped = False
            error_distance = np.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)
            error_angle = np.arctan2(self.goal_y - self.y, self.goal_x - self.x) - self.theta
            error_angle = np.arctan2(np.sin(error_angle), np.cos(error_angle))
            index_goal = int(np.rint(np.rad2deg(error_angle) + 180))
            goal_view = self.get_view(self.lidar.ranges, index_goal, 12, error_distance)
            obj_distance = min(self.lidar.ranges[:180])
            a1 = 0
            a2 = 0
            if obj_distance < 999999:
                a1 = self.lidar.ranges.index(obj_distance) -180
                a2 = a1 + 90
                if obj_distance < 0.2:
                    a2 += 10
                if obj_distance > 0.3:
                    a2 -= 5
                
            follow_angle = np.deg2rad(a2)

            if goal_view == "clear" or error_distance < obj_distance:
                self.follow = False
            elif goal_view == "obstacle" and obj_distance <= self.follow_distance and not self.begin_follow:
                self.follow = True
                self.begin_follow = True

            if self.follow:
                if np.abs(follow_angle) < self.turning_d:
                    angular_speed = self.Kp_angular * follow_angle
                    self.robot_vel.linear.x = 0.1
                    self.robot_vel.angular.z = 0.0
                else:
                    angular_speed = self.Kp_angular * follow_angle
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
            
            else:
                if error_distance < self.stop_d:
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                    goal_reached = True
                    stopped = True
                elif np.absolute(error_angle) < self.turning_d:
                    linear_speed = self.Kp_linear * error_distance
                    angular_speed = self.Kp_angular * error_angle
                    self.robot_vel.linear.x = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)
                    self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
                else:
                    angular_speed = self.Kp_angular * error_angle
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

            if obj_distance < self.stop_d:
                stopped = True
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
            
            '''
            self.t.header.stamp = self.get_clock().now().to_msg()
            self.t.header.frame_id = 'base_footprint'
            self.t.child_frame_id = 'goal_frame'
            self.t.transform.translation.x = error_distance * np.cos(error_angle)
            self.t.transform.translation.y = error_distance * np.sin(error_angle)
            self.t.transform.translation.z = 0.0
            q = transforms3d.euler.euler2quat(0, 0, error_angle)
            self.t.transform.rotation.x = q[1]
            self.t.transform.rotation.y = q[2]
            self.t.transform.rotation.z = q[3]
            self.t.transform.rotation.w = q[0]
            self.tf_br2.sendTransform(self.t)
            '''
            self.pub_v.publish(self.robot_vel)
            self.inf1.data = str(
                str(goal_view == "obstacle") + " " +
                str(self.follow) + " " +
                "ind:" + str(index_goal) + " " +
                "o_d:" + str(np.round(obj_distance, 2)) + " o_th:" + str(a1) + " " +
                "f_th:" + str(a2) + " " +
                "x:" + str(np.round(self.x, 2)) + " y:" + str(np.round(self.y, 2)) + " th:" + str(np.round(np.rad2deg(self.theta), 2)) + " " +
                "g_x:" + str(np.round(self.goal_x, 2)) + " g_y:" + str(np.round(self.goal_y, 2)) + " " +
                "e_d:" + str(np.round(error_distance, 2)) + " e_th:" + str(np.round(np.rad2deg(error_angle), 2)) + " " +
                str(goal_reached) + " " +
                str(stopped)
            )
            self.inf2.data = str(np.round(np.array(self.ll), 2)) +  " " + str(np.round(error_distance, 2)) + " " + str(np.round(np.rad2deg(error_angle), 2)) + " " + str(index_goal) + " " + str(goal_view)
            self.pub.publish(self.inf1)
            self.pub2.publish(self.inf2)

def main(args=None):
    rclpy.init(args=args)
    node2 = ControllerClass()
    try:
        rclpy.spin(node2)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node2.destroy_node()

if __name__ == '__main__':
    main()
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point 
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import transforms3d

#   You are looking for this comand
#   ros2 topic pub /goal geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}" --once 

class point_stabilisation_controller(Node):
    def __init__(self):
        super().__init__('point_stabilisation_controller')
        
        # Parámetros del nodo
        self.declare_parameter('kp_linear', 0.2)
        self.declare_parameter('kp_angular', 0.1)
        self.declare_parameter('max_linear_speed', 0.55)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('goal_tolerance', 0.1)
        self.declare_parameter('angular_tolerance', math.radians(5))  # 5 grados en radianes
        
        
        
        # Parámetros del controlador
        self.kp_linear = self.get_parameter('kp_linear').get_parameter_value().double_value
        self.kp_angular = self.get_parameter('kp_angular').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.angular_tolerance = self.get_parameter('angular_tolerance').get_parameter_value().double_value      
        
        # Estado del robot
        self.current_pose = Point()
        self.current_yaw = 0.0
        self.goal_pose = None
        self.goal_reached = False
        self.orientation_locked = False  # Nuevo estado de orientación
        self.obstacle_near = False  # Estado de proximidad a obstáculos
        # Subsciptores
        self.odom_sub = self.create_subscription(
            Odometry,
            'ground_truth',
            self.odom_callback,
            10)
            
        self.goal_sub = self.create_subscription(
            Point,
            'goal',
            self.goal_callback,
            10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        
        # Timer de control
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
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
            self.detener_robot()
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
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def scan_callback(self, msg):
        front_angles = range(-30, 30)
        min_distance = min([msg.ranges[i] for i in front_angles if not math.isinf(msg.ranges[i])])
        self.obstacle_near = min_distance < 0.5
        
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
'''
