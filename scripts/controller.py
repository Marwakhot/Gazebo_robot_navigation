#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from fuzzy_brain import FuzzyNavigator

GOAL_X = -3.5
GOAL_Y = 3.5
GOAL_THRESHOLD = 0.4

class Controller:
    def __init__(self):
        print("=" * 60)
        print("FUZZY LOGIC ROBOT NAVIGATION")
        print("=" * 60)
        rospy.init_node('fuzzy_controller_node')
        
        self.brain = FuzzyNavigator()
        
        self.sub_scan = rospy.Subscriber('/robot/laser/scan', LaserScan, self.scan_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.x = -4.0
        self.y = -4.0
        self.yaw = 0.0
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0
        
        self.scan_received = False
        self.odom_received = False
        
        print("Waiting for sensor data...")
        rate = rospy.Rate(10)
        timeout = 0
        while (not self.scan_received or not self.odom_received) and timeout < 100:
            rate.sleep()
            timeout += 1
        
        if timeout >= 100:
            print("ERROR: Timeout waiting for sensors!")
        else:
            print(f"Sensors connected! Start: ({self.x:.2f}, {self.y:.2f}) Goal: ({GOAL_X}, {GOAL_Y})")
            print("=" * 60)

    def odom_callback(self, msg):
        self.odom_received = True
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def scan_callback(self, msg):
        self.scan_received = True
        ranges = np.array(msg.ranges)
        ranges[ranges == float('inf')] = 10.0
        ranges[ranges == 0.0] = 10.0
        ranges = np.nan_to_num(ranges, nan=10.0)
        
        # Front: -40 to +40 degrees
        front_ranges = np.concatenate([ranges[320:], ranges[:40]])
        # Left: 40 to 140 degrees
        left_ranges = ranges[40:140]
        # Right: 220 to 320 degrees
        right_ranges = ranges[220:320]
        
        self.front_dist = np.min(front_ranges) if len(front_ranges) > 0 else 10.0
        self.left_dist = np.min(left_ranges) if len(left_ranges) > 0 else 10.0
        self.right_dist = np.min(right_ranges) if len(right_ranges) > 0 else 10.0

    def get_heading_error(self):
        desired_yaw = math.atan2(GOAL_Y - self.y, GOAL_X - self.x)
        error_yaw = desired_yaw - self.yaw
        
        if error_yaw > math.pi:
            error_yaw -= 2 * math.pi
        elif error_yaw < -math.pi:
            error_yaw += 2 * math.pi
            
        return math.degrees(error_yaw)

    def run(self):
        rate = rospy.Rate(10)
        print("Starting Navigation...")
        last_print = 0
        
        try:
            while not rospy.is_shutdown():
                dist_to_goal = math.sqrt((GOAL_X - self.x)**2 + (GOAL_Y - self.y)**2)
                
                if dist_to_goal < GOAL_THRESHOLD:
                    print("\n" + "=" * 60)
                    print("GOAL REACHED!")
                    print(f"Final position: ({self.x:.2f}, {self.y:.2f})")
                    print("=" * 60)
                    stop_cmd = Twist()
                    self.pub_cmd.publish(stop_cmd)
                    break

                heading_err = self.get_heading_error()
                
                speed, turn_rate = self.brain.get_action(
                    self.front_dist, 
                    self.left_dist, 
                    self.right_dist, 
                    heading_err
                )
                
                cmd = Twist()
                cmd.linear.x = float(speed)
                cmd.angular.z = float(turn_rate)
                self.pub_cmd.publish(cmd)
                
                current_time = rospy.Time.now().to_sec()
                if current_time - last_print > 2.0:
                    print(f"Pos: ({self.x:.2f}, {self.y:.2f}) | Goal dist: {dist_to_goal:.2f}m | "
                          f"Heading err: {heading_err:.1f}deg | Front: {self.front_dist:.2f}m | "
                          f"Speed: {speed:.2f} | Turn: {turn_rate:.2f}")
                    last_print = current_time
                
                rate.sleep()
        
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            stop_cmd = Twist()
            self.pub_cmd.publish(stop_cmd)
            print("Robot stopped")

if __name__ == '__main__':
    try:
        Controller().run()
    except rospy.ROSInterruptException:
        pass
    