#!/usr/bin/env python3
# Behavior Tree for Robot Navigation - FIXED v5

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence, Selector

GOAL_X = -3.5
GOAL_Y = 3.5
GOAL_TOLERANCE = 0.3
OBSTACLE_DISTANCE = 0.75
MAX_LINEAR = 0.4
MAX_ANGULAR = 0.8
LASER_FRONT_WINDOW = 40

class RobotState:
    def __init__(self):
        self.odom = None
        self.laser = None

robot_state = RobotState()
cmd_pub = None

def publish_twist(linear_x=0.0, angular_z=0.0):
    global cmd_pub
    if cmd_pub is None:
        return
    t = Twist()
    t.linear.x = max(-MAX_LINEAR, min(MAX_LINEAR, linear_x))
    t.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular_z))
    cmd_pub.publish(t)

def get_robot_pose():
    od = robot_state.odom
    if od is None:
        return None
    x = od.pose.pose.position.x
    y = od.pose.pose.position.y
    q = od.pose.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    return x, y, yaw

def front_min_distance():
    scan = robot_state.laser
    if scan is None:
        return float('inf')
    left = scan.ranges[:LASER_FRONT_WINDOW]
    right = scan.ranges[-LASER_FRONT_WINDOW:]
    front = list(left) + list(right)
    cleaned = [d for d in front if not (math.isinf(d) or math.isnan(d))]
    if not cleaned:
        return float('inf')
    return min(cleaned)

def get_side_distances():
    scan = robot_state.laser
    if scan is None:
        return float('inf'), float('inf')
    
    left_indices = range(60, 120)
    left = [scan.ranges[i] for i in left_indices if i < len(scan.ranges)]
    left_clean = [d for d in left if not (math.isinf(d) or math.isnan(d))]
    left_min = min(left_clean) if left_clean else float('inf')
    
    right_indices = range(240, 300)
    right = [scan.ranges[i] for i in right_indices if i < len(scan.ranges)]
    right_clean = [d for d in right if not (math.isinf(d) or math.isnan(d))]
    right_min = min(right_clean) if right_clean else float('inf')
    
    return left_min, right_min

class IsGoalReached(Behaviour):
    def __init__(self, name="IsGoalReached"):
        super(IsGoalReached, self).__init__(name)

    def update(self):
        pose = get_robot_pose()
        if pose is None:
            return Status.FAILURE
        
        x, y, _ = pose
        dist = math.hypot(GOAL_X - x, GOAL_Y - y)
        
        if dist <= GOAL_TOLERANCE:
            publish_twist(0.0, 0.0)
            print("=" * 60)
            print(f"GOAL REACHED! Position: ({x:.2f}, {y:.2f}), Distance: {dist:.2f}m")
            print("=" * 60)
            return Status.SUCCESS
        
        return Status.FAILURE

class IsObstacleNear(Behaviour):
    def __init__(self, name="IsObstacleNear"):
        super(IsObstacleNear, self).__init__(name)

    def update(self):
        dist = front_min_distance()
        if dist < OBSTACLE_DISTANCE:
            print(f"Obstacle detected at {dist:.2f}m")
            return Status.SUCCESS
        return Status.FAILURE

class AvoidObstacle(Behaviour):
    def __init__(self, name="AvoidObstacle"):
        super(AvoidObstacle, self).__init__(name)
        self.started = False
        self.start_time = None
        self.turn_direction = 1.0
        self.phase = 0

    def initialise(self):
        self.started = True
        self.start_time = rospy.Time.now()
        self.phase = 0
        
        left_dist, right_dist = get_side_distances()
        
        if left_dist > right_dist:
            self.turn_direction = 1.0
            print(f"AVOIDING - TURNING LEFT")
        else:
            self.turn_direction = -1.0
            print(f"AVOIDING - TURNING RIGHT")

    def update(self):
        if not self.started:
            return Status.FAILURE
        
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        front_dist = front_min_distance()
        
        # Phase 0: Turn sharply for 3 seconds OR until clear
        if self.phase == 0:
            if elapsed < 3.5 and front_dist < OBSTACLE_DISTANCE * 1.5:
                publish_twist(linear_x=0.0, angular_z=self.turn_direction * 1.0)
                return Status.RUNNING
            else:
                self.phase = 1
                self.start_time = rospy.Time.now()
                print("PHASE 1: MOVING FORWARD")
        
        # Phase 1: Move forward for 5 SECONDS
        if self.phase == 1:
            elapsed_phase1 = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed_phase1 < 5.0:
                if front_dist < OBSTACLE_DISTANCE:
                    # Still blocked, turn more
                    publish_twist(linear_x=0.2, angular_z=self.turn_direction * 0.6)
                else:
                    # Clear, go forward
                    publish_twist(linear_x=0.4, angular_z=0.0)
                return Status.RUNNING
            else:
                self.phase = 2
                self.start_time = rospy.Time.now()
                print("PHASE 2: TURNING BACK TOWARD GOAL")
        
        # Phase 2: Turn back toward goal while moving for 4 SECONDS
        if self.phase == 2:
            elapsed_phase2 = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed_phase2 < 4.0:
                publish_twist(linear_x=0.35, angular_z=-self.turn_direction * 0.5)
                return Status.RUNNING
            else:
                self.phase = 3
                self.start_time = rospy.Time.now()
                print("PHASE 3: FINAL FORWARD")
        
        # Phase 3: Final forward for 3 SECONDS
        if self.phase == 3:
            elapsed_phase3 = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed_phase3 < 3.0:
                publish_twist(linear_x=0.4, angular_z=0.0)
                return Status.RUNNING
            else:
                publish_twist(0.0, 0.0)
                self.started = False
                print("OBSTACLE AVOIDED - RESUMING NAVIGATION")
                return Status.SUCCESS
        
        return Status.RUNNING

    def terminate(self, new_status):
        self.started = False
        self.phase = 0

class MoveTowardsGoal(Behaviour):
    def __init__(self, name="MoveTowardsGoal"):
        super(MoveTowardsGoal, self).__init__(name)
        self.last_print = 0

    def update(self):
        pose = get_robot_pose()
        if pose is None:
            return Status.FAILURE
        
        x, y, yaw = pose
        
        goal_angle = math.atan2(GOAL_Y - y, GOAL_X - x)
        angle_diff = goal_angle - yaw
        
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        distance = math.hypot(GOAL_X - x, GOAL_Y - y)
        
        front_dist = front_min_distance()
        if front_dist < OBSTACLE_DISTANCE:
            return Status.FAILURE
        
        angular_vel = 3.0 * angle_diff
        angular_vel = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular_vel))
        
        if abs(angle_diff) > 0.5:
            linear_vel = 0.1
        elif abs(angle_diff) > 0.2:
            linear_vel = 0.25
        else:
            linear_vel = MAX_LINEAR
        
        publish_twist(linear_x=linear_vel, angular_z=angular_vel)
        
        current_time = rospy.Time.now().to_sec()
        if current_time - self.last_print > 2.0:
            print(f"Position: ({x:.2f}, {y:.2f}) | Distance to goal: {distance:.2f}m | Heading error: {math.degrees(angle_diff):.1f} deg")
            self.last_print = current_time
        
        return Status.RUNNING

def create_behavior_tree():
    root = Selector(name="NavigationRoot", memory=False)
    
    goal_check = IsGoalReached(name="GoalCheck")
    
    obstacle_avoidance = Sequence(name="ObstacleAvoidance", memory=False)
    obstacle_avoidance.add_children([
        IsObstacleNear(name="ObstacleCheck"),
        AvoidObstacle(name="Avoid")
    ])
    
    navigation = MoveTowardsGoal(name="Navigate")
    
    root.add_children([
        goal_check,
        obstacle_avoidance,
        navigation
    ])
    
    return root

def odom_callback(msg):
    robot_state.odom = msg

def laser_callback(msg):
    robot_state.laser = msg

def main():
    global cmd_pub
    
    print("=" * 60)
    print("BEHAVIOR TREE ROBOT NAVIGATION")
    print("=" * 60)
    
    rospy.init_node('behavior_tree_navigation')
    print("ROS node initialized")
    
    cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/robot/laser/scan', LaserScan, laser_callback)
    print("Publishers and subscribers ready")
    
    print("Waiting for sensor data...")
    rate = rospy.Rate(10)
    timeout = 0
    while (robot_state.odom is None or robot_state.laser is None) and timeout < 100:
        rate.sleep()
        timeout += 1
    
    if timeout >= 100:
        print("Timeout waiting for sensors!")
        return
    
    print("Sensors connected")
    
    print("Building behavior tree...")
    root = create_behavior_tree()
    
    x = robot_state.odom.pose.pose.position.x
    y = robot_state.odom.pose.pose.position.y
    
    print("\n" + "=" * 60)
    print(f"Starting navigation: ({x:.2f}, {y:.2f}) -> ({GOAL_X}, {GOAL_Y})")
    print(f"Goal tolerance: {GOAL_TOLERANCE}m")
    print("=" * 60 + "\n")
    
    rate = rospy.Rate(10)
    
    try:
        while not rospy.is_shutdown():
            root.tick_once()
            
            if root.status == Status.SUCCESS:
                print("\n" + "=" * 60)
                print("NAVIGATION COMPLETE! GOAL REACHED!")
                print("=" * 60)
                publish_twist(0.0, 0.0)
                break
            
            rate.sleep()
    
    except KeyboardInterrupt:
        print("\nNavigation interrupted by user")
    
    finally:
        publish_twist(0.0, 0.0)
        print("Robot stopped")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass