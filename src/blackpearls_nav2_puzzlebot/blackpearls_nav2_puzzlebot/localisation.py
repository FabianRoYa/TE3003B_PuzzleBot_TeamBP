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
        
        # self.declare_parameter('initialPose',[0,0,0])
        
        # Create subscribers
        self.wr_sub = self.create_subscription(
            Float32, self.get_parameter('wr').value, self.wr_callback, qos.qos_profile_sensor_data)
        self.wl_sub = self.create_subscription(
            Float32,self.get_parameter('wl').value, self.wl_callback, qos.qos_profile_sensor_data)

        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, 'ground_truth', 10)
        # self.tf_broadcaster = TransformBroadcaster(self)
        # self.wr_pub = self.create_publisher(Float32, 'wr_loc', qos.qos_profile_sensor_data)
        # self.wl_pub = self.create_publisher(Float32, 'wl_loc', qos.qos_profile_sensor_data)

        # Robot constants
        self.r = 0.05    # Wheel radius [m]
        self.L = 0.19    # Wheel separation [m]

        # State variables
        self.x = 0.3     # Position x [m]
        self.y = 2.75     # Position y [m]
        self.theta = 0.0 # Orientation [rad]
        self.wr = 0.0    # Right wheel speed [rad/s]
        self.wl = 0.0    # Left wheel speed [rad/s]
        
        ## NO TOCAR !!!
        # Covariance parameters
        self.P = np.diag([0.01, 0.01, 0.01])  # 3x3 covariance matrix
        self.A = 8.85e-5   # Varianza promedio x e y (0.000885 m²)
        self.B = -4.6e-6   # Covarianza xy (-0.000046 m²)
        self.C = 6.08e-5   # Varianza theta (0.000608 rad²)
        
        
        # Timing control
        self.prev_time = self.get_clock().now().nanoseconds
        self.get_logger().info("Localisation node started")

        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.prev_time) * 1e-9
        
        # Calculate velocities
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        
        # Update pose and covariance (UPDATED)
        self.update_pose(v, w, dt)
        self.update_covariance(v, w, dt)  # NEW METHOD
        
        # Update time
        self.prev_time = current_time 
        
        # Publish odometry
        self.publish_odometry()
        # self.publish_wheels()

    # NEW METHOD: Covariance propagation    
    def update_covariance(self, v, w,dt):
        # Jacobian matrices
        J_h = np.array([
            [1, 0, -v * dt * np.sin(self.theta)],
            [0, 1, v * dt * np.cos(self.theta)],
            [0, 0, 1]
        ])
        
        Q = np.array([
            [self.A,self.B,self.B],
            [self.B,self.A,self.B],
            [self.B,self.B,self.C]
        ])
        # Covariance propagation
        self.P = J_h @ self.P @ J_h.T + Q
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
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'base_footprint'
        
        # # Position
        # odom_msg.pose.pose.position.x = self.x
        # odom_msg.pose.pose.position.y = self.y
        # odom_msg.pose.pose.position.z = 0.05
        
        # # Orientation
        # q = transforms3d.euler.euler2quat(0, 0, self.theta)
        # odom_msg.pose.pose.orientation.x = q[1]
        # odom_msg.pose.pose.orientation.y = q[2]
        # odom_msg.pose.pose.orientation.z = q[3]
        # odom_msg.pose.pose.orientation.w = q[0]

        # Covariance matrix (NEW SECTION)
        odom_msg.pose.covariance = [0.0]*36
        odom_msg.pose.covariance[0] = self.P[0,0]  # var x
        odom_msg.pose.covariance[7] = self.P[1,1]   # var y
        odom_msg.pose.covariance[35] = self.P[2,2]  # var theta
        odom_msg.pose.covariance[1] = self.P[0,1]   # cov xy
        odom_msg.pose.covariance[6] = self.P[1,0]   # cov yx
        odom_msg.pose.covariance[5] = self.P[0,2]   # cov xθ
        odom_msg.pose.covariance[30] = self.P[2,0]  # cov θx
        odom_msg.pose.covariance[11] = self.P[1,2]  # cov yθ
        odom_msg.pose.covariance[31] = self.P[2,1]  # cov θy

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