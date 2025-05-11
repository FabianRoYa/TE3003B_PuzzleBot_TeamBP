#! /usr/bin/env python3
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node



class PublishWheelSpeeds(Node):

  def __init__(self):
    super().__init__('publish_wr_wl')
    self.create_subscription(Odometry, 'wheel/odometry', self.odom_cb, 10 )
    # Create wr publisher
    self.pub_wr = self.create_publisher(Float32, 'VelocityEncR', 10)
    # Create wr publisher
    self.pub_wl = self.create_publisher(Float32, 'VelocityEncL', 10)

    #Robot parameters
    self.L = 0.52 # wheel separation [m]
    self.r = 0.14 # radius of the wheels in [m]
    self.wr = Float32()
    self.wl = Float32()

    # Call on_timer function every second
    self.dt= 0.04
    self.timer = self.create_timer(self.dt, self.on_timer)

  def on_timer(self):
    self.pub_wr.publish(self.wr)
    self.pub_wl.publish(self.wl)

  def odom_cb(self, msg):
    v = msg.twist.twist.linear.x
    w = msg.twist.twist.angular.z
    [self.wr.data, self.wl.data] = self.get_wheel_speeds(v, w)

  def get_wheel_speeds(self, v, w):
    #Get the wheel speeds from robot velocity
    wr = (2*v + w * self.L)/(2*self.r)
    wl = (2*v - w * self.L)/(2*self.r)
    return [wr, wl]



def main(args=None): 
  rclpy.init(args=args) 
  f_p=PublishWheelSpeeds() 
  rclpy.spin(f_p) 
  f_p.destroy_node() 
  rclpy.shutdown() 

     

if __name__ == '__main__': 
    main()