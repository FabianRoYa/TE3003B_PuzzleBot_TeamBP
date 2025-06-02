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
from rclpy.parameter import Parameter
from builtin_interfaces.msg import Time

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Configuration parameters
        self.wheel_radius = 0.05

        # Joint state initialization
        self.joint_state = JointState()
        self.joint_state.name = ['wheel_left_joint', 'wheel_right_joint']
        self.joint_state.position = [0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0]        
        
        # Setup publishers and timers
        self.wr = 0.0
        self.wl = 0.0
        self.wr_sub = self.create_subscription(Float32, 'VelocityEncR', self.wr_callback, qos.qos_profile_sensor_data)
        self.wl_sub = self.create_subscription(Float32, 'VelocityEncL', self.wl_callback, qos.qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.last_time = self.get_clock().now()  # Para cálculo correcto de dt

    def wr_callback(self, msg):
        self.wr = msg.data / self.wheel_radius

    def wl_callback(self, msg):
        self.wl = msg.data / self.wheel_radius

    def timer_callback(self):
        # Calcular dt preciso
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        
        # Actualizar posiciones de las ruedas
        self.joint_state.position[0] += self.wl * dt
        self.joint_state.position[1] += self.wr * dt
        
        # Mantener posiciones en rango [-π, π]
        self.joint_state.position = [
            (pos + np.pi) % (2 * np.pi) - np.pi 
            for pos in self.joint_state.position
        ]
        
        # Publicar estados de las articulaciones
        self.joint_state.velocity = [self.wl, self.wr]
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.joint_state)

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