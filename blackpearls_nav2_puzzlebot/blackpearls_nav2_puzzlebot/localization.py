import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np

class LocalizationClass(Node):
    def __init__(self):
        super().__init__('localization')
        timer_period = 0.1
        self.r = 0.052
        self.L = 0.19
        self.robot_v = 0.0
        self.robot_w = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.w_r = 0.0
        self.w_l = 0.0
        self.begin = True
        self.robot_pose = Odometry()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sub_wr = self.create_subscription(Float32, 'wr', self.wr_callback, 10)
        self.sub_wl = self.create_subscription(Float32, 'wl', self.wl_callback, 10)
        self.pub_p = self.create_publisher(Odometry, 'robot_pose', 10)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        if self.begin:
            self.previous_time = self.get_clock().now().nanoseconds
            self.begin = False
        else:
            self.current_time = self.get_clock().now().nanoseconds
            dt = float(self.current_time - self.previous_time)/(10.0 ** 9)
            self.previous_time = self.current_time
            [self.robot_v,self.robot_w] = self.get_robot_vel(self.w_r, self.w_l)
            self.robot_x = self.robot_x + self.robot_v * np.cos(self.robot_theta) * dt
            self.robot_y = self.robot_y + self.robot_v * np.sin(self.robot_theta) * dt
            self.robot_theta = self.robot_theta + self.robot_w * dt
            self.robot_pose.pose.pose.position.x = self.robot_x
            self.robot_pose.pose.pose.position.y = self.robot_y
            self.robot_pose.pose.pose.position.z = 0.0
            q = transforms3d.euler.euler2quat(0, 0, self.robot_theta)
            self.robot_pose.pose.pose.orientation.x = q[1]
            self.robot_pose.pose.pose.orientation.y = q[2]
            self.robot_pose.pose.pose.orientation.z = q[3]
            self.robot_pose.pose.pose.orientation.w = q[0]
            self.pub_p.publish(self.robot_pose)

    def wr_callback(self, msg):
        self.w_r = msg.data

    def wl_callback(self, msg):
        self.w_l = msg.data

    def get_robot_vel(self, wr, wl):
        v = self.r * (wr + wl) / 2.0
        w = (self.r / self.L) * (wr - wl)
        return[v, w]

def main(args=None):
    rclpy.init(args=args)
    node1 = LocalizationClass()
    try:
        rclpy.spin(node1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node1.destroy_node()

if __name__ == '__main__':
    main()