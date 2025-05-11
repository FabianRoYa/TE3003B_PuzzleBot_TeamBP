import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np

class ControllerClass(Node):
    def __init__(self):
        super().__init__('controller')
        timer_period = 0.1
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.Kp_linear = 0.1
        self.Kp_angular = 1.0
        self.max_linear_speed = 2.0
        self.max_angular_speed = 0.8
        self.linear_tolerance = 0.01
        self.angular_tolerance = np.pi / 180.0
        self.angular_limit = np.pi / 18.0
        self.v = 0.0
        self.w = 0.0
        self.robot_vel = Twist()
        self.next_point = Bool()
        self.inf1 = String()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sub_p = self.create_subscription(Odometry, 'robot_pose', self.pose_callback, 10)
        self.sub_g = self.create_subscription(PoseStamped, 'set_point', self.goal_callback, 10)
        self.pub_v = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_n = self.create_publisher(Bool, 'next_point', 10)
        self.pub = self.create_publisher(String, 'inf1', 10)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        self.next_point.data = False
        error_distance = np.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)
        error_angle = np.arctan2(self.goal_y - self.y, self.goal_x - self.x) - self.theta
        error_angle = np.arctan2(np.sin(error_angle), np.cos(error_angle))
        if error_distance < self.linear_tolerance and error_angle < self.angular_tolerance:
            self.next_point.data = True
            self.v = 0.0
            self.w = 0.0
        elif np.absolute(error_angle) < self.angular_limit:
            linear_speed = self.Kp_linear * error_distance
            angular_speed = self.Kp_angular * error_angle
            self.v = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)
            self.w = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
        else:
            angular_speed = self.Kp_angular * error_angle
            self.v = 0.0
            self.w = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
        self.robot_vel.linear.x = self.v
        self.robot_vel.angular.z = self.w
        self.inf1.data = str(
            "g_x: " + str(np.round(self.goal_x, 2)) + " " +
            "g_y: " + str(np.round(self.goal_y, 2)) + " " +
            "x: " + str(np.round(self.x, 2)) + " " +
            "y: " + str(np.round(self.y, 2)) + " " +
            "th: " + str(np.round(np.rad2deg(self.theta), 2)) + " " +
            "e_d: " + str(np.round(error_distance, 2)) + " " +
            "e_th: " + str(np.round(np.rad2deg(error_angle), 2)) + " " +
            "\n")
        self.pub_v.publish(self.robot_vel)
        self.pub_n.publish(self.next_point)
        self.pub.publish(self.inf1)

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = transforms3d.euler.quat2euler([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])

    def goal_callback(self, msg1):
        self.goal_x = msg1.pose.position.x
        self.goal_y = msg1.pose.position.y

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