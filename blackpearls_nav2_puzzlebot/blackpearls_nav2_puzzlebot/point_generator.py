import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
import numpy as np

class PointGeneratorClass(Node):
    def __init__(self):
        super().__init__('point_generator')
        self.declare_parameter('figure', 0)
        timer_period = 0.1
        self.next = False
        self.prev = False
        self.n = 3
        self.end = 7
        self.r = 0.5
        self.index = 0
        self.figure = 0
        self.figure_prev = 0
        self.points = self.points_make()
        self.goal = PoseStamped()
        self.inf1 = String()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sub_n = self.create_subscription(Bool, 'next_point', self.next_callback, 10)
        self.pub_g = self.create_publisher(PoseStamped, 'set_point', 10)
        self.pub = self.create_publisher(String, 'inf', 10)
        self.start_time = self.get_clock().now()

    def points_make(self):
        n = self.n
        points = []
        while n <= self.end:
            fraction = 2.0 * np.pi / float(n)
            l = []
            for i in range(n):
                x = float(((np.cos((i+1) * fraction) * self.r) - self.r) * -1)
                y = float(np.sin((i+1) * fraction) * self.r)
                l.append([x, y])
            for i in range(self.end - n):
                l.append([0.0, 0.0])
            points.append(l)
            n += 1
        return (points)

    def timer_callback(self):
        self.figure = self.get_parameter('figure').get_parameter_value().integer_value
        if self.figure != self.figure_prev:
            self.index = 0
        if self.next == True and self.prev == False:
            self.index += 1
        if self.figure >= self.n and self.figure <= self.end:
            self.goal.pose.position.x = self.points[self.figure - 3][self.index][0]
            self.goal.pose.position.y = self.points[self.figure - 3][self.index][1]
            self.pub_g.publish(self.goal)
        self.inf1.data = str(self.figure)
        self.pub.publish(self.inf1)
        self.prev = self.next
        self.figure_prev = self.figure

    def next_callback(self, msg):
        self.next = bool(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node3 = PointGeneratorClass()
    try:
        rclpy.spin(node3)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node3.destroy_node()

if __name__ == '__main__':
    main()