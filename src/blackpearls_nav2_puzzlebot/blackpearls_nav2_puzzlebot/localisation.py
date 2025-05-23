import rclpy 
from rclpy.node import Node 
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32 
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy import qos 
import numpy as np 
import transforms3d 

class Localisation(Node): 

    def __init__(self): 
        super().__init__('localisation') 
        
        self.declare_parameter('wr', 'wr')
        self.declare_parameter('wl', 'wl')

        # self.declare_parameter('initialPose',[0.0,0.0,0.0])

        # Create subscribers
        self.wr_sub = self.create_subscription(
            Float32, self.get_parameter('wr').value, self.wr_callback, qos.qos_profile_sensor_data)
        self.wl_sub = self.create_subscription(
            Float32,self.get_parameter('wl').value, self.wl_callback, qos.qos_profile_sensor_data)

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.wr_pub = self.create_publisher(Float32, 'wr_loc', qos.qos_profile_sensor_data)
        self.wl_pub = self.create_publisher(Float32, 'wl_loc', qos.qos_profile_sensor_data)

        # Robot constants
        self.r = 0.05    # Wheel radius [m]
        self.L = 0.19    # Wheel separation [m]

        # State variables
        self.x = 0.0  # Position x [m]
        self.y = 0.0  # Position y [m]
        self.theta = 0.0  # Orientation [rad]
        self.wr = 0.0    # Right wheel speed [rad/s]
        self.wl = 0.0    # Left wheel speed [rad/s]


        # Timing control
        self.prev_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Localisation node started")

        self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.prev_time) * 1e-9
        
        # Calculate velocities
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        
        # Update pose and covariance (UPDATED)
        self.update_pose(v, w, dt)
        
        # Update time
        self.prev_time = current_time 
        
        # Publish odometry
        self.publish_odometry()
        self.publish_wheels()


    def wr_callback(self, msg):
        self.wr = msg.data

    def wl_callback(self, msg):
        self.wl = msg.data

    def update_pose(self, v, w, dt):
        # Update position and orientation
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += w * dt
        # Normalize angle
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        
    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.00
        
        # Orientation
        q = transforms3d.euler.euler2quat(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]
        odom_msg.pose.pose.orientation.w = q[0]

     

        self.odom_pub.publish(odom_msg)

    def publish_wheels(self):
        self.wr_pub.publish(Float32(data=self.wr))
        self.wl_pub.publish(Float32(data=self.wl))

def main(args=None):
    rclpy.init(args=args)
    node = Localisation()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()