import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from std_msgs.msg import Int32, Float32, String
import numpy as np
import math
import transforms3d
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

class localisation(Node):
    def __init__(self):
        super().__init__('localisation')
        
        # Parámetros configurables
        self.declare_parameter('wr_topic', 'wr')
        self.declare_parameter('wl_topic', 'wl')
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.19)
        self.declare_parameter('camera_frame', 'camera_link_optical')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('marker_frame_prefix', 'aruco_')

        # Un Snickers a quien pueda hacerlo parametros
        # Tienen que estar en order de aparición.
        '''
        self.marker_positions = {
            # 1: [1.88,0.25,0.0],   # 1: [0.25, 1.88, 0.0],    # Marcador 1 en x=1.5m, y=0.5m, orientación 90°
            2: [0.25,1.88,0.0],    # 2: [2.7, 0.08, 0.0],      # Marcador 2 en x=2.0m, y=1.0m
            3: [2.25,3.125,0.0],   # 3: [2.0, 1.825, 0.0],    # Marcador 3 en x=2.0m, y=1.5m
            # 4: [0.09,0.85,0.0],   # 4: [0.09, 0.85, 0.0],     # Marcador 4 en x=0.5m, y=1.5m
            # 5: [0.09,0.85,0.0],   # 5: [0.09, 0.85, 0.0],    # Marcador 5 en x=0.5m, y=2.0m
        }
        '''
        self.marker_positions = [
            [2.0, 1.825, 0.0],
            [0.25, 1.88, 0.0],
            [2.25, 3.125, 0.0],
            [2.7, 0.08, 0.0],
            [0.09, 0.85, 0.0]
        ]

        # Obtener parámetros
        wr_topic = self.get_parameter('wr_topic').value
        wl_topic = self.get_parameter('wl_topic').value
        initial_pose = self.get_parameter('initial_pose').value
        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.marker_frame_prefix = self.get_parameter('marker_frame_prefix').value
        
        # self.marker_positions = self.get_parameter('marker_positions').value
        self.T_base_cam = np.array([[0, 0, 1, 0.1241], [-1, 0, 0, 0], [0, -1, 0, 0.117], [0, 0, 0, 1]])

        # Estado del robot [x, y, theta]
        self.x = np.array(initial_pose).reshape(3, 1)
        self.P = np.diag([0.01, 0.01, 0.01])  # Covarianza inicial
        #self.P = np.diag([0.5, 0.5, 0.3])

        # Matrices de ruido
        self.Q = np.diag([0.01, 0.01, 0.01])  # Ruido del proceso
        #self.R = np.diag([0.05, 0.05, 0.05])  # Ruido de medición
        #self.Q = np.array([[0.5, 0.01, 0.01], [0.01, 0.5, 0.01], [0.01, 0.01, 0.2]])
        self.R = np.array([[0.04, 0], [0, 0.01]])
        
        # Variables de control
        self.id = 0
        self.wr = 0.0
        self.wl = 0.0
        self.last_time = self.get_clock().now().nanoseconds
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Suscriptores
        self.sub_id = self.create_subscription(Int32, 'id', self.id_callback, 10)
        self.wr_sub = self.create_subscription(Float32, wr_topic, self.wr_callback, qos_profile_sensor_data)
        self.wl_sub = self.create_subscription(Float32, wl_topic, self.wl_callback, qos_profile_sensor_data)
        
        self.inf = String()
        self.pub = self.create_publisher(String, 'inf', 10)

        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, 'ground_truth', 10)
        self.wr_pub = self.create_publisher(Float32, 'wr_loc', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl_loc', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Temporizador principal
        self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info("EKF Localisation node started")

    def id_callback(self, msg):
        self.id = msg.data

    def wr_callback(self, msg):
        self.wr = msg.data
        #self.wr_pub.publish(msg)

    def wl_callback(self, msg):
        self.wl = msg.data
        #self.wl_pub.publish(msg)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds
        dt = float(current_time - self.last_time)/(10.0 ** 9)
        
        # Paso 1: Predicción con odometría
        self.predict_with_odometry(dt)
        
        # Paso 2: Intentar corrección con marcadores visibles
        self.try_marker_correction()
        
        # Paso 3: Publicar estado actualizado
        self.publish_odometry()
        
        self.publish_wheels()
        
        self.last_time = current_time

    def predict_with_odometry(self, dt):
        # Calcular velocidades
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.L
        theta = self.x[2, 0]
        
        # Actualizar estado
        self.x[0, 0] += v * np.cos(theta) * dt
        self.x[1, 0] += v * np.sin(theta) * dt
        self.x[2, 0] += w * dt
        self.x[2, 0] = np.arctan2(np.sin(self.x[2, 0]), np.cos(self.x[2, 0]))
        
        # Actualizar covarianza
        H = np.array([[1, 0, -np.sin(theta) * v * dt], [0, 1, np.cos(theta) * v * dt], [0, 0, 1]])
        self.P = H @ self.P @ np.transpose(H) + self.Q
        self.inf.data = "x: " + str(np.round(self.x, 2))

    def try_marker_correction(self):
        if self.id != -1:
            trans = self.tf_buffer.lookup_transform(self.camera_frame, 'aruco', rclpy.time.Time())
            marker_pose = self.marker_positions[self.id - 1]
            self.apply_marker_correction(trans, marker_pose)

    def apply_marker_correction(self, transform, marker_pose):
        # Obtener transformación de cámara a marcador
        t = transform.transform.translation
        q = transform.transform.rotation
        
        # Convertir a matriz de rotación
        roll, pitch, yaw = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])
        rot_matrix = transforms3d.euler.euler2mat(roll, pitch, yaw)
        
        # Construir matriz de transformación homogénea
        T_cam_marker = np.eye(4)
        T_cam_marker[:3, :3] = rot_matrix
        T_cam_marker[0, 3] = t.x
        T_cam_marker[1, 3] = t.y
        T_cam_marker[2, 3] = t.z
        
        T_base_marker = self.T_base_cam @ T_cam_marker

        # Obtener posición esperada del marcador en el mundo
        marker_x, marker_y, _ = marker_pose

        z1 = np.array([[np.sqrt(T_base_marker[0, 3] ** 2 + T_base_marker[1, 3] ** 2)], [np.arctan2(T_base_marker[1, 3], T_base_marker[0, 3])]])
        z2 = np.array([[np.sqrt((marker_x - self.x[0, 0]) ** 2 + (marker_y - self.x[1, 0]) ** 2)], [np.arctan2((marker_y - self.x[1, 0]), (marker_x - self.x[0, 0])) - self.x[2, 0]]])
        z2[1][0] = np.arctan2(np.sin(z2[1][0]), np.cos(z2[1][0]))
        dx = marker_x - self.x[0, 0]
        dy = marker_y - self.x[1, 0]
        p = dx ** 2 + dy ** 2
        G = np.array([[-dx/np.sqrt(p), -dy/np.sqrt(p), 0], [dy/p, -dx/p, -1]])
        Z = G @ self.P @ np.transpose(G) + self.R
        K = self.P @ np.transpose(G) @ np.linalg.inv(Z)
        self.x = self.x + K @ (z1 - z2)
        self.P = (np.identity(3) - K @ G) @ self.P
        self.inf.data = "z1: " + str(np.round(z1, 2)) + "z2: " + str(np.round(z2, 2)) + "mx: " + str(marker_x) + "my: " + str(marker_y) + "x: " + str(np.round(self.x, 2))

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.base_frame
        
        # Posición
        odom.pose.pose.position.x = self.x[0, 0]
        odom.pose.pose.position.y = self.x[1, 0]
        odom.pose.pose.position.z = 0.0
        #self.get_logger().info(f'Posición: {odom.pose.pose.position.x:.2f}, {odom.pose.pose.position.y:.2f}')
        # Orientación
        q = transforms3d.euler.euler2quat(0, 0, self.x[2, 0])
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]
        #self.get_logger().info(f'Orientación: {odom.pose.pose.orientation.w:.2f}')
        # Covarianza
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = self.P[0, 0]  # xx
        odom.pose.covariance[1] = self.P[0, 1]  # xy
        odom.pose.covariance[5] = self.P[0, 2]  # xθ
        odom.pose.covariance[6] = self.P[1, 0]  # yx
        odom.pose.covariance[7] = self.P[1, 1]  # yy
        odom.pose.covariance[11] = self.P[1, 2]  # yθ
        odom.pose.covariance[30] = self.P[2, 0]  # θx
        odom.pose.covariance[31] = self.P[2, 1]  # θy
        odom.pose.covariance[35] = self.P[2, 2]  # θθ
        
        self.odom_pub.publish(odom)
        
        # Publicar transformación TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x[0, 0]
        t.transform.translation.y = self.x[1, 0]
        #t.transform.rotation = odom.pose.pose.orientation
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        self.tf_broadcaster.sendTransform(t)

        self.pub.publish(self.inf)
        
    def publish_wheels(self):
        wr_msg = Float32()
        wr_msg.data = self.wr
        self.wr_pub.publish(wr_msg)

        wl_msg = Float32()
        wl_msg.data = self.wl
        self.wl_pub.publish(wl_msg)

def main(args=None):
    rclpy.init(args=args)
    node = localisation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()