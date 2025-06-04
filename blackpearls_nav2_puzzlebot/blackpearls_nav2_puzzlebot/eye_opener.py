import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class EyeOpener(Node):
    def __init__(self):
        super().__init__('eye_opener')
        
        # Params for the launch file
        self.declare_parameter('camera_topic','camera')
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)


        # set parameters
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        
        # Publishers and subscribers
        self.open_eye_pub = self.create_publisher(Image, self.camera_topic, 10)

        self.bridge = CvBridge()
        self.eye = cv2.VideoCapture(self._gestreamer_pipeline(frame_rate=30, width=self.image_width, height=self.image_height), cv2.CAP_GSTREAMER)
        
        self.get_logger().info(f'Opening camera on topic: {self.camera_topic} with resolution {self.image_width}x{self.image_height}')
        
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.eye.read()
        if ret:
            image_resize = cv2.resize(frame,(self.image_width,self.image_height))
            self.open_eye_pub.publish(self.bridge.cv2_to_imgmsg(image_resize, 'bgr8'))
        else:
            self.get_logger().error('Failed to read from camera')

def main():
    rclpy.init()
    node= EyeOpener()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().info('Node interrupted. Shutting down...')
        node.get_logger().info(f'Error: {e}')
        if rclpy.ok():
            rclpy.shutdown()
    finally:
        node.eye.release()
        cv2.destroyAllWindows()
        node.destroy_node()

if __name__ == '__main__':
    main()