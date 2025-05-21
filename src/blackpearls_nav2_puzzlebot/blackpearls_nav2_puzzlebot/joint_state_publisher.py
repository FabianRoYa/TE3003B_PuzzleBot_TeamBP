import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np
import rclpy.qos as qos
from std_msgs.msg import Float32

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        
        self.namespace = self.get_namespace().rstrip('/')
        
        # Declare parameters
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('odometry_frame', 'odom')

        # Configuration parameters
        self.wheel_radius = 0.05  # Ensure this matches your robot's wheel radius
        self.base_height = 0.05
        
        # Frames 
        self.odomFrame = self.get_parameter('odometry_frame').get_parameter_value().string_value.strip('/')
        
        # Setup publishers and timers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.002, self.timer_callback)
        self.publish_static_transforms()
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
        )
        self.wr_sub = self.create_subscription(
            Float32, 
            'wr_loc', 
            self.wr_callback, 
            qos.qos_profile_sensor_data
        )
        self.wl_sub = self.create_subscription(
            Float32, 
            'wl_loc', 
            self.wl_callback, 
            qos.qos_profile_sensor_data
        )
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.q = None
        self.wr = 0.0  # Angular velocity (rad/s)
        self.wl = 0.0  # Angular velocity (rad/s)
        
        # Joint state initialization
        self.joint_state = JointState()
        self.joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]  # Will be updated dynamically
        self.joint_state.effort = []
        
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def wr_callback(self, msg):
        # Convert linear velocity (m/s) to angular velocity (rad/s)
        self.wr = msg.data / self.wheel_radius

    def wl_callback(self, msg):
        # Convert linear velocity (m/s) to angular velocity (rad/s)
        self.wl = msg.data / self.wheel_radius

    def odom_callback(self, msg):
        initial_pose = self.get_parameter('initial_pose').get_parameter_value().double_array_value
        #self.x = initial_pose[0] + msg.pose.pose.position.x
        #self.y = initial_pose[1] + msg.pose.pose.position.y
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.q = msg.pose.pose.orientation

    def publish_static_transforms(self):
        static_transforms = self.create_transform(
            parent_frame='map',
            child_frame=self.odomFrame,
            x=0.0, y=0.0, z=0.0,
            roll=0.0, pitch=0.0, yaw=0.0
        )
        self.tf_static_broadcaster.sendTransform(static_transforms)

    def publish_dynamic_transforms(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.start_time
        self.start_time = current_time
        
        # Update joint positions
        self.joint_state.position[0] += self.wl * dt
        self.joint_state.position[1] += self.wr * dt
        
        # Keep positions within [-π, π] for visualization (optional)
        self.joint_state.position = [
            (pos + np.pi) % (2 * np.pi) - np.pi 
            for pos in self.joint_state.position
        ]
        
        # Update joint velocities
        self.joint_state.velocity = [self.wl, self.wr]
        
        # Publish joint states
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)
        
        # Publish base_link transform
        dynamic_transform = TransformStamped()
        dynamic_transform.header.stamp = self.get_clock().now().to_msg()
        dynamic_transform.header.frame_id = f'{self.namespace}/odom'
        dynamic_transform.child_frame_id = f'{self.namespace}/base_footprint'
        
        dynamic_transform.transform.translation.x = self.x
        dynamic_transform.transform.translation.y = self.y
        dynamic_transform.transform.translation.z = 0.0
        
        if self.q:
            dynamic_transform.transform.rotation = self.q
        else:
            dynamic_transform.transform.rotation.w = 1.0  # Default to no rotation
        
        self.tf_broadcaster.sendTransform(dynamic_transform)

    def timer_callback(self):
        self.publish_dynamic_transforms()

    def create_transform(self, parent_frame, child_frame, x, y, z, roll, pitch, yaw):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        
        q = transforms3d.euler.euler2quat(roll, pitch, yaw)
        transform.transform.rotation.x = q[1]
        transform.transform.rotation.y = q[2]
        transform.transform.rotation.z = q[3]
        transform.transform.rotation.w = q[0]
        
        return transform

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
