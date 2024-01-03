import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import time
import sys
import numpy as np

NAV = 1
TRACK = 0
nav_goal = 0  # the goal point
sys_state = NAV
lidar_update_time = 0
navigator = None

class Nav_Tracker(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)


    def nav_pose(self):
        global nav_goal
        global sys_state
        global navigator
            
        points = np.zeros((7,3))
        points[0] = [-4.000, -3.500, 0.0] # P2
        points[1] = [-2.980, -2.506, 0.0] # P2 center
        points[2] = [0.0154, -3.771, 0.0] # P3
        points[3] = [-0.760, -3.548, 0.0] # P3 center
        points[4] = [0.252, 0.429, 0.0] # P4
        points[5] = [-0.005, -0.018, 0.0] # P4 center
        points[6] = [-3.809, 0.508, 0.0] # P1
        if sys_state == NAV:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = points[nav_goal][0]
            goal_pose.pose.position.y = points[nav_goal][1]
            theta = points[nav_goal][2] * 3.1415926 / 180.0
            goal_pose.pose.orientation.w = np.cos(theta / 2.0)
            goal_pose.pose.orientation.z = np.sin(theta / 2.0)
            navigator.goToPose(goal_pose)
            while not navigator.isTaskComplete():
                pass
            print("Arrived at the goal!")
            nav_goal += 1
            if nav_goal % 2 == 0:
                sys_state = TRACK
        if nav_goal >= len(points):
            sys.exit()
    
    def scan_cb(self, msg):
        global nav_goal
        global sys_state
        global lidar_update_time
        
        if lidar_update_time < 5:
            lidar_update_time += 1
            print('lidar_update_time:', lidar_update_time, end='\r')
            return
        
        if sys_state == TRACK:
            range_min = 10.0
            for i in range(len(msg.ranges)):
                if msg.ranges[i] < range_min and msg.ranges[i] != 0.0:
                    range_min = msg.ranges[i]
                    range_min_index = i
                    range_min_angle = msg.angle_min + i * msg.angle_increment
                    range_min_angle = range_min_angle * 180.0 / 3.1415926
                    if range_min_angle > 180:
                        range_min_angle = range_min_angle - 360  # -180 ~ 180
            print('min:', range_min, 'index: ', range_min_index, 'angle: ', range_min_angle)

            range_threshold = 0.2
            angle_threshold = 10
            twist = Twist()
            if range_min > range_threshold:
                if abs(range_min_angle) < angle_threshold:
                    twist.linear.x = 0.3
            if abs(range_min_angle) > angle_threshold:
                twist.angular.z = range_min_angle/180.0 * 1.0 + np.sign(range_min_angle) * 1.0
            
            print('linear.x:', twist.linear.x, 'angular.z:', twist.angular.z)
            self.vel_pub.publish(twist)
            if range_min < range_threshold and abs(range_min_angle) < angle_threshold:
                sys_state = NAV
                print('Arrived at the pole!')
                time.sleep(2.0)
        else: 
            self.nav_pose()
            lidar_update_time = 0
        pass
        

def main(args=None):
    global navigator
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -3.809
    initial_pose.pose.position.y = 0.508
    initial_pose.pose.orientation.z = -0.009804245494456227
    initial_pose.pose.orientation.w = 0.9999519372301273
    navigator.setInitialPose(initial_pose)
    nav_tracker = Nav_Tracker()
    rclpy.spin(nav_tracker)
    nav_tracker.destroy_node()
    rclpy.shutdown()
#   ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}

if __name__ == '__main__':
    main()