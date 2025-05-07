import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rclpy import qos
import numpy as np

class PuzzlebotSim(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim')
        
        # Create a subscriber to the /cmd_vel topic 
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10) 

        # Create a publisher for the right and left wheel velocities 
        self.wr_pub = self.create_publisher(Float32,'wr',qos.qos_profile_sensor_data) 
        self.wl_pub = self.create_publisher(Float32,'wl',qos.qos_profile_sensor_data) 

        ############ ROBOT CONSTANTS ################  
        self.r=0.05 #puzzlebot wheel radius [m] 
        self.L = 0.19 #puzzlebot wheel separation [m] 

        ############ Variables ############### 
        self.w = 0.0 # robot's angular speed [rad/s] 
        self.v = 0.0 #robot's linear speed [m/s] 
        self.wr_msg = Float32() #Ros message to publish the right wheel speed 
        self.wl_msg = Float32() #Ros message to publish the left wheel speed 


        timer_period = 0.02 # Desired time to update the robot's pose [s] 
        # Create a timer to publish the wheel speeds 
        self.timer = self.create_timer(timer_period, self.timer_callback) 

        # WRITE YOUR CODE HERE 

     

    def timer_callback(self): 
        # Update the robot's pose based on the current velocities
        self.wr_msg.data, self.wl_msg.data = self.get_wheel_speeds(self.v, self.w)
        # Publish the wheel speeds
        self.wr_pub.publish(self.wr_msg)
        self.wl_pub.publish(self.wl_msg)
     

    def cmd_vel_callback(self, msg): 

        # Get the linear and angular velocities from the message 
        self.v = msg.linear.x 
        self.w = msg.angular.z 


    def get_wheel_speeds(self, v, w): 
        # Calculate the wheel speeds based on the linear and angular velocities 
        wr = (2*v + w*self.L) / (2 * self.r)
        wl = (2*v - w*self.L)/ (2 * self.r)
        
        # print(f'wR:{wr} wL:{wl}')
    
        return wr, wl 

def main():
    rclpy.init()
    node = PuzzlebotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()