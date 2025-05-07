import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry  # Corrected import
import transforms3d
import numpy as np
import rclpy.qos as qos
from std_msgs.msg import Float32

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # Configuration parameters
        self.wheel_radius = 0.05  # Same as localization node
        self.base_height = 0.05   # Height from base_link to ground
        
        # Setup publishers and timers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.02, self.timer_callback)
        self.publish_static_transforms()
        
        # Subscribers
        # Corrected subscription to Odometry
        self.odom_sub = self.create_subscription(
            Odometry,  # Changed from JointState to Odometry
            '/odom',
            self.odom_callback,
            qos.qos_profile_sensor_data
            
        )
        self.wr_sub = self.create_subscription(
            Float32, 
            'wr', 
            self.wr_callback, 
            qos.qos_profile_sensor_data
        )
        self.wl_sub = self.create_subscription(
            Float32, 
            'wl', 
            self.wl_callback, 
            qos.qos_profile_sensor_data
        )
        
        self.x = 0.0
        self.y = 0.0
        self.wr = 0.0
        self.wl = 0.0
        # self.theta = 0.0
        self.q = None
        # Joint state initialization
        self.joint_state = JointState()
        self.joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0]
        
        self.start_time = self.get_clock().now().nanoseconds /1e9
    def quaternion_to_yaw(self, q):
        # Convert geometry_msgs/Quaternion to yaw angle
        quat = [q.x, q.y, q.z, q.w]
        euler = transforms3d.euler.quat2euler(quat, 'sxyz')
        return euler[2]  # Yaw is the third component

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y  # Corrected from .x to .y
        self.q = msg.pose.pose.orientation
        # q = msg.pose.pose.orientation
        # self.theta = self.quaternion_to_yaw(q)
        if msg.pose.pose.orientation is not None:
            self.q = msg.pose.pose.orientation
        else:
            self.get_logger().warn("Orientación no recibida, usando valor por defecto")
        
    def wr_callback(self, msg):
        self.wr = msg.data
        
    def wl_callback(self, msg):
        self.wl = msg.data
        
    def publish_static_transforms(self):
        static_transforms = [
            self.create_transform(
                parent_frame = 'map',
                child_frame= 'odom',
                x=0.0, 
                y=0.0, 
                z=0.0,
                roll=0.0,
                pitch=0.0,
                yaw=0.0
            ),
            self.create_transform(
                parent_frame="base_link",
                child_frame="base_footprint",
                x=0.0, 
                y=0.0, 
                z=self.base_height,
                roll=0.0,
                pitch=0.0,
                yaw=0.0
            )
            
        ]
        self.tf_static_broadcaster.sendTransform(static_transforms)

    def publish_dinamic_transforms(self):
        
        if None in [self.x, self.y, self.q]:
            self.get_logger().warn("Datos incompletos, omitiendo transformación")
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.start_time
        self.start_time=current_time    
        
        # Update joint positions and velocities with real data
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.position[0] = (self.joint_state.position[0] + self.wl * dt) % (2 * np.pi)
        self.joint_state.position[1] = (self.joint_state.position[1] + self.wr * dt) % (2 * np.pi)
        dynamic_transform = TransformStamped()
        dynamic_transform.header.stamp = self.get_clock().now().to_msg()
        dynamic_transform.header.frame_id = 'odom'
        dynamic_transform.child_frame_id = 'base_link'
        dynamic_transform.transform.translation.x = self.x
        dynamic_transform.transform.translation.y = self.y
        dynamic_transform.transform.translation.z = self.base_height
        
        dynamic_transform.transform.rotation.x = self.q.x
        dynamic_transform.transform.rotation.y = self.q.y
        dynamic_transform.transform.rotation.z = self.q.z
        dynamic_transform.transform.rotation.w = self.q.w
     
        # Update joint velocities        
        self.joint_pub.publish(self.joint_state)
        self.tf_broadcaster.sendTransform(dynamic_transform)
        

    def timer_callback(self):
        self.publish_dinamic_transforms()
        
        

    def create_transform(self, parent_frame, child_frame, 
                        x, y, z, roll, pitch, yaw):
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