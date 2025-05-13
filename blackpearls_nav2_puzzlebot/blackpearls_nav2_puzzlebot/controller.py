import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
#from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, PoseStamped #, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import transforms3d
import numpy as np

class ControllerClass(Node):
    def __init__(self):
        super().__init__('controller')
        timer_period = 0.1
        self.goal_x = 0.3
        self.goal_y = 2.7
        self.x = 0.3
        self.y = 2.7
        self.theta = 0.0
        self.Kp_linear = 0.1
        self.Kp_angular = 0.3
        self.max_linear_speed = 0.3
        self.max_angular_speed = 0.3
        self.follow_distance = 0.8
        self.stop_d = 0.05
        self.turning_d = np.deg2rad(5)
        self.follow = False
        self.lidar_recieved = False
        #self.tf_br2 = TransformBroadcaster(self)
        #self.t = TransformStamped()
        self.lidar = LaserScan()
        #self.goal_lidar = LaserScan()
        self.robot_vel = Twist()
        self.next_point = Bool()
        self.inf1 = String()
        self.inf2 = String()
        self.ll = []
        self.zero = (np.zeros(360, dtype=np.float32) + 0.05).tolist()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sub_p = self.create_subscription(Odometry, 'robot_pose', self.pose_callback, 10)
        self.sub_g = self.create_subscription(PoseStamped, 'set_point', self.goal_callback, 10)
        self.sub_l = self.create_subscription(LaserScan, "scan", self.lidar_callback, 10)
        #self.pub_l = self.create_publisher(LaserScan, 'goal_scan', 10)
        self.pub_v = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub = self.create_publisher(String, 'inf1', 10)
        self.pub2 = self.create_publisher(String, 'inf2', 10)
        self.start_time = self.get_clock().now()

    def get_view(self, r, center, limit, max_distance):
        #self.goal_lidar.ranges = self.zero
        self.ll = []
        detect = []
        i_i = center - limit
        i_f = center + limit
        if i_i < 0:
            for i in range(0, i_f):
                self.ll.append(r[i])
                #self.goal_lidar.ranges[i] = r[i]
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
            for i in range(360 - i_i, 360):
                self.ll.append(r[i])
                #self.goal_lidar.ranges[i] = r[i]
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
        elif i_f > 360:
            for i in range(i_i, 360):
                self.ll.append(r[i])
                #self.goal_lidar.ranges[i] = r[i]
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
            for i in range(0, i_f - 360):
                self.ll.append(r[i])
                #self.goal_lidar.ranges[i] = r[i]
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
        else:
            for i in range(i_i, i_f):
                self.ll.append(r[i])
                #self.goal_lidar.ranges[i] = r[i]
                if r[i] > max_distance:
                    detect.append(0)
                else:
                    detect.append(1)
        if np.sum(detect) == 0:
            view = "clear"
        else:
            view = "obstacle"
        return view

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.theta = transforms3d.euler.quat2euler([msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z])

    def goal_callback(self, msg1):
        self.goal_x = msg1.pose.position.x
        self.goal_y = msg1.pose.position.y

    def lidar_callback(self, msg2):
        self.lidar = msg2
        #self.goal_lidar = msg2
        self.lidar_recieved = True

    def timer_callback(self):
        if self.lidar_recieved:
            goal_reached = False
            stopped = False
            error_distance = np.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)
            error_angle = np.arctan2(self.goal_y - self.y, self.goal_x - self.x) - self.theta
            error_angle = np.arctan2(np.sin(error_angle), np.cos(error_angle))
            index_goal = int(np.rint(np.rad2deg(error_angle) + 180))
            goal_view = self.get_view(self.lidar.ranges, index_goal, 10, error_distance)
            obj_distance = min(self.lidar.ranges[:180])
            a1 = 0
            a2 = 0
            if obj_distance < 999999:
                a1 = self.lidar.ranges.index(obj_distance) -180
                a2 = a1 + 90
                if obj_distance < 0.2:
                    a2 += 10
                if obj_distance > 0.3:
                    a2 -= 5
                
            follow_angle = np.deg2rad(a2)

            if goal_view == "clear" or error_distance < obj_distance:
                self.follow = False
            elif goal_view == "obstacle" and obj_distance <= self.follow_distance:
                self.follow = True

            if self.follow:
                #if np.abs(follow_angle) < self.turning_d and obj_distance > 0.15:
                if np.abs(follow_angle) < self.turning_d:
                    angular_speed = self.Kp_angular * follow_angle
                    self.robot_vel.linear.x = 0.1
                    self.robot_vel.angular.z = 0.0
                else:
                    angular_speed = self.Kp_angular * follow_angle
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
            
            else:
                if error_distance < self.stop_d:
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = 0.0
                    goal_reached = True
                    stopped = True
                elif np.absolute(error_angle) < self.turning_d:
                    linear_speed = self.Kp_linear * error_distance
                    angular_speed = self.Kp_angular * error_angle
                    self.robot_vel.linear.x = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)
                    self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)
                else:
                    angular_speed = self.Kp_angular * error_angle
                    self.robot_vel.linear.x = 0.0
                    self.robot_vel.angular.z = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

            if obj_distance < self.stop_d:
                stopped = True
                self.robot_vel.linear.x = 0.0
                self.robot_vel.angular.z = 0.0
            
            '''
            self.t.header.stamp = self.get_clock().now().to_msg()
            self.t.header.frame_id = 'base_footprint'
            self.t.child_frame_id = 'goal'
            self.t.transform.translation.x = error_distance * np.cos(error_angle)
            self.t.transform.translation.y = error_distance * np.cos(error_angle)
            self.t.transform.translation.z = 0.0
            q = transforms3d.euler.euler2quat(0, 0, error_angle)
            self.t.transform.rotation.x = q[1]
            self.t.transform.rotation.y = q[2]
            self.t.transform.rotation.z = q[3]
            self.t.transform.rotation.w = q[0]
            self.tf_br2.sendTransform(self.t)
            '''

            self.pub_v.publish(self.robot_vel)
            self.inf1.data = str(
                str(goal_view == "obstacle") + " " +
                str(self.follow) + " " +
                "ind:" + str(index_goal) + " " +
                "o_d:" + str(np.round(obj_distance, 2)) + " o_th:" + str(a1) + " " +
                "f_th:" + str(a2) + " " +
                "x:" + str(np.round(self.x, 2)) + " y:" + str(np.round(self.y, 2)) + " th:" + str(np.round(np.rad2deg(self.theta), 2)) + " " +
                "g_x:" + str(np.round(self.goal_x, 2)) + " g_y:" + str(np.round(self.goal_y, 2)) + " " +
                "e_d:" + str(np.round(error_distance, 2)) + " e_th:" + str(np.round(np.rad2deg(error_angle), 2)) + " " +
                str(goal_reached) + " " +
                str(stopped)
            )
            self.inf2.data = str(np.round(np.array(self.ll), 2)) +  " " + str(np.round(error_distance, 2)) + " " + str(np.round(np.rad2deg(error_angle), 2)) + " " + str(index_goal) + " " + str(goal_view)
            self.pub.publish(self.inf1)
            self.pub2.publish(self.inf2)
            #self.pub_l.publish(self.goal_lidar)

def main(args=None):
    rclpy.init(args=args)
    node2 = ControllerClass()
    try:
        rclpy.spin(node2)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node2.destroy_node()

if __name__ == '__main__':
    main()
