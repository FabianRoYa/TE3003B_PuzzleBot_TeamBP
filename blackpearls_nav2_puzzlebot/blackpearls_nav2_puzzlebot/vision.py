import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
import cv2
from cv2 import aruco
import numpy as np
import transforms3d
from cv_bridge import CvBridge



class VisionClass(Node):
    def __init__(self):
        super().__init__('vision')
        timer_period = 0.1
        
                # CAMBIO: Usar valores float en lugar de int
        self.declare_parameter('camera_matrix', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('distortion_coefficients', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('marker_length', 0.14)
        
        self.img = []
        self.image_received = False
        
        # CAMBIO: Usar double_array_value en lugar de double_array_value (ya es float)
        self.cam_m = np.array(
            self.get_parameter('camera_matrix').get_parameter_value().double_array_value
        ).reshape((3, 3))
        
        self.cam_d = np.array(
            self.get_parameter('distortion_coefficients').get_parameter_value().double_array_value
        ).reshape((5, 1))
        
        self.markerLength = self.get_parameter('marker_length').get_parameter_value().double_value

        # API LEGACY para OpenCV 4.2.0
        self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()
        
        self.bridge = CvBridge()
        self.inf = String()
        self.id = Int32()
        self.t = TransformStamped()
        self.tf_br1 = TransformBroadcaster(self)
        self.sub_image = self.create_subscription(Image, 'camera', self.camera_callback, 10)
        self.pub_image1 = self.create_publisher(Image, 'image_process', 10)
        self.pub = self.create_publisher(String, 'inf', 10)
        self.pub_id = self.create_publisher(Int32, 'id', 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def rot2rpy(self, R):
        yaw = np.arctan2(R[1, 0],R[0, 0])
        pitch = np.arctan2(-R[2, 0],R[0, 0]*np.cos(yaw)+R[1, 0]*np.sin(yaw))
        roll = np.arctan2(-R[1, 2]*np.cos(yaw)+R[0, 2]*np.sin(yaw),R[1, 1]*np.cos(yaw)-R[0, 1]*np.sin(yaw))
        return roll, pitch, yaw

    def camera_callback(self, msg1):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg1, 'bgr8')            
            self.image_received = True
            
        except Exception as e:
            self.get_logger().error(f'Failed to get image: {e}')
            self.image_received = False

    def timer_callback(self):
        self.id.data = 0
        if self.image_received:
            img_mod = self.img.copy()
            gray = cv2.cvtColor(img_mod, cv2.COLOR_BGR2GRAY)
            
            # Detección de marcadores con API legacy
            markerCorners, markerIds, _ = aruco.detectMarkers(
                gray, 
                self.dictionary,
                parameters=self.parameters
            )
            
            if markerIds is not None and len(markerIds) > 0:
                # Estimación de pose para todos los marcadores
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    markerCorners, 
                    self.markerLength, 
                    self.cam_m, 
                    self.cam_d
                )
                
                for i in range(len(markerIds)):
                    rvec = rvecs[i]
                    tvec = tvecs[i]
                    distance = np.linalg.norm(tvec)
                    corners = markerCorners[i].reshape((4, 2))
                    topLeft, _, bottomRight, _ = corners
                    cX = int((topLeft[0] + bottomRight[0]) /2.0)
                    cY = int((topLeft[1] + bottomRight[1]) /2.0)
                    
                    # Dibujar marcadores y ejes
                    aruco.drawDetectedMarkers(img_mod, markerCorners, markerIds) 
                    cv2.drawFrameAxes(img_mod, self.cam_m, self.cam_d, rvec, tvec, 0.1)
                    cv2.putText(img_mod, f"{distance:.2f} m", (cX, cY-15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
                    
                    if float(distance) <= 1.5:
                        rot, _ = cv2.Rodrigues(rvec)
                        roll, pitch, yaw = self.rot2rpy(rot)
                        self.t.header.stamp = self.get_clock().now().to_msg()
                        self.t.header.frame_id = 'camera_link_optical'
                        self.t.child_frame_id = f'aruco_{markerIds[i][0]}'
                        q = transforms3d.euler.euler2quat(roll, pitch, yaw)
                        self.t.transform.translation.x = tvec[0][0]
                        self.t.transform.translation.y = tvec[0][1]
                        self.t.transform.translation.z = tvec[0][2]
                        self.t.transform.rotation.x = q[1]
                        self.t.transform.rotation.y = q[2]
                        self.t.transform.rotation.z = q[3]
                        self.t.transform.rotation.w = q[0]
                        self.id.data = int(markerIds[i][0])
                        if markerIds[i][0] >= 5:
                            self.id.data = -1
                            self.tf_br1.sendTransform(self.t)
            self.pub_image1.publish(self.bridge.cv2_to_imgmsg(img_mod, 'bgr8'))
        self.pub_id.publish(self.id)

def main(args=None):
    rclpy.init(args=args)
    node = VisionClass()
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