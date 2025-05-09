import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Float32
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState
import transforms3d
import numpy as np

class FramePublisherClass(Node):
    def __init__(self):
        super().__init__('frame')
        timer_period = 0.1
        self.r = 0.052
        self.L = 0.19
        self.robot_v = 0.0
        self.robot_w = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.r_theta = 0.0
        self.l_theta = 0.0
        self.current_time = 0.0
        self.previous_time = 0.0
        self.begin = True
        self.t = TransformStamped()
        self.tf_br1 = TransformBroadcaster(self)
        self.ctrlJoints = JointState()
        self.w_r = Float32()
        self.w_l = Float32()
        self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
        self.ctrlJoints.name = ["wheel_r_joint", "wheel_l_joint"]
        self.ctrlJoints.position = [0.0] * 2
        self.ctrlJoints.velocity = [0.0] * 2
        self.ctrlJoints.effort = [0.0] * 2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sub_v = self.create_subscription(Twist, 'cmd_vel', self.vel_callback, 10)
        self.pub_s = self.create_publisher(JointState, 'joint_states', 10)
        self.pub_wr = self.create_publisher(Float32, 'wr', 10)
        self.pub_wl = self.create_publisher(Float32, 'wl', 10)
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        if self.begin:
            self.previous_time = self.get_clock().now().nanoseconds
            self.begin = False
        else:
            self.current_time = self.get_clock().now().nanoseconds
            dt = float(self.current_time - self.previous_time)/(10.0 ** 9)
            self.previous_time = self.current_time
            self.t.header.stamp = self.get_clock().now().to_msg()
            self.t.header.frame_id = 'odom'
            self.t.child_frame_id = 'base_footprint'
            wr = (2 * self.robot_v + self.L * self.robot_w) / (2 * self.r)
            wl = (2 * self.robot_v - self.L * self.robot_w) / (2 * self.r)
            self.robot_x = self.robot_x + self.robot_v * np.cos(self.robot_theta) * dt
            self.robot_y = self.robot_y + self.robot_v * np.sin(self.robot_theta) * dt
            self.robot_theta = self.robot_theta + self.robot_w * dt
            self.r_theta = self.r_theta + wr * dt
            self.l_theta = self.l_theta + wl * dt
            self.t.transform.translation.x = self.robot_x
            self.t.transform.translation.y = self.robot_y
            self.t.transform.translation.z = 0.0
            q = transforms3d.euler.euler2quat(0, 0, self.robot_theta)
            self.t.transform.rotation.x = q[1]
            self.t.transform.rotation.y = q[2]
            self.t.transform.rotation.z = q[3]
            self.t.transform.rotation.w = q[0]
            self.ctrlJoints.header.stamp = self.get_clock().now().to_msg()
            self.ctrlJoints.position[0] = self.r_theta
            self.ctrlJoints.position[1] = self.l_theta
            self.w_r.data = wr
            self.w_l.data = wl
            self.tf_br1.sendTransform(self.t)
            self.pub_s.publish(self.ctrlJoints)
            self.pub_wr.publish(self.w_r)
            self.pub_wl.publish(self.w_l)

    def vel_callback(self, msg):
        self.robot_v = msg.linear.x
        self.robot_w = msg.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = FramePublisherClass()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()