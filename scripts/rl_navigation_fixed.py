#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import pickle
import os

class SimpleQLearningAgent:
    """Simple Q-Learning agent for robot navigation"""
    
    def __init__(self, n_states=100, n_actions=5):
        self.n_states = n_states
        self.n_actions = n_actions
        
        self.q_table = np.zeros((n_states, n_actions))
        
        self.learning_rate = 0.1
        self.discount_factor = 0.95
        self.epsilon = 1.0
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        
        self.actions = [
            [0.3, 0.0],
            [0.3, 0.5],
            [0.3, -0.5],
            [0.0, 0.8],
            [0.0, -0.8],
        ]
    
    def discretize_state(self, lidar_data, distance_to_goal, angle_to_goal):
        front_distance = np.mean(lidar_data[0:3])
        
        if front_distance < 0.5:
            distance_bin = 0
        elif front_distance < 1.0:
            distance_bin = 1
        elif front_distance < 2.0:
            distance_bin = 2
        elif front_distance < 3.0:
            distance_bin = 3
        else:
            distance_bin = 4
        
        if angle_to_goal < -math.pi/4:
            angle_bin = 0
        elif angle_to_goal < math.pi/4:
            angle_bin = 1
        elif angle_to_goal < 3*math.pi/4:
            angle_bin = 2
        else:
            angle_bin = 3
        
        if distance_to_goal < 1.0:
            goal_dist_bin = 0
        elif distance_to_goal < 3.0:
            goal_dist_bin = 1
        elif distance_to_goal < 5.0:
            goal_dist_bin = 2
        elif distance_to_goal < 8.0:
            goal_dist_bin = 3
        else:
            goal_dist_bin = 4
        
        state = distance_bin * 20 + angle_bin * 5 + goal_dist_bin
        return min(state, self.n_states - 1)
    
    def choose_action(self, state):
        if np.random.random() < self.epsilon:
            return np.random.randint(self.n_actions)
        else:
            return np.argmax(self.q_table[state])
    
    def update(self, state, action, reward, next_state):
        best_next_action = np.argmax(self.q_table[next_state])
        td_target = reward + self.discount_factor * self.q_table[next_state][best_next_action]
        td_error = td_target - self.q_table[state][action]
        self.q_table[state][action] += self.learning_rate * td_error
    
    def decay_epsilon(self):
        self.epsilon = max(self.epsilon_min, self.epsilon * self.epsilon_decay)
    
    def save(self, filename="/root/robot_project/scripts/q_table.pkl"):
        with open(filename, 'wb') as f:
            pickle.dump(self.q_table, f)
        print(f"Q-table saved to {filename}")
    
    def load(self, filename="/root/robot_project/scripts/q_table.pkl"):
        if os.path.exists(filename):
            with open(filename, 'rb') as f:
                self.q_table = pickle.load(f)
            print(f"Q-table loaded from {filename}")
            return True
        return False


class RobotNavigationEnv:
    """Environment wrapper for robot in Gazebo"""
    
    def __init__(self):
        rospy.init_node('simple_rl_navigation', anonymous=True)
        
        self.robot_x = -4.0
        self.robot_y = -4.0
        self.robot_theta = 0.0
        self.lidar_data = np.ones(24) * 10.0
        self.lidar_received = False
        self.odom_received = False
        
        self.goal_x = 2.0
        self.goal_y = 3.0
        
        print("Setting up ROS publishers and subscribers...")
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lidar_sub = rospy.Subscriber('/robot/laser/scan', LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        print("Waiting for sensor data...")
        rate = rospy.Rate(10)
        timeout = 0
        while (not self.lidar_received or not self.odom_received) and timeout < 100:
            rate.sleep()
            timeout += 1
        
        if timeout >= 100:
            print("\n WARNING: Timeout waiting for sensors!")
        else:
            print("Sensors connected successfully!")
    
    def lidar_callback(self, data):
        self.lidar_received = True
        ranges = np.array(data.ranges)
        ranges = np.nan_to_num(ranges, nan=10.0, posinf=10.0, neginf=0.0)
        num_points = len(ranges)
        if num_points > 0:
            step = max(1, num_points // 24)
            self.lidar_data = ranges[::step][:24]
            if len(self.lidar_data) < 24:
                self.lidar_data = np.pad(self.lidar_data, (0, 24 - len(self.lidar_data)), constant_values=10.0)
    
    def odom_callback(self, data):
        self.odom_received = True
        self.robot_x = data.pose.pose.position.x
        self.robot_y = data.pose.pose.position.y
        
        orientation = data.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.robot_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def get_distance_to_goal(self):
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        return math.sqrt(dx**2 + dy**2)
    
    def get_angle_to_goal(self):
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - self.robot_theta
        return math.atan2(math.sin(angle_diff), math.cos(angle_diff))
    
    def execute_action(self, action):
        cmd = Twist()
        cmd.linear.x = action[0]
        cmd.angular.z = action[1]
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(0.2)
    
    def calculate_reward(self, prev_distance):
        distance = self.get_distance_to_goal()
        reward = 0.0
        done = False
        
        if prev_distance is not None:
            reward += (prev_distance - distance) * 5.0
        
        if distance < 0.5:
            reward += 100.0
            done = True
            print("Goal reached!")
        
        min_distance = np.min(self.lidar_data)
        if min_distance < 0.3:
            reward -= 20.0
            done = True
            print("Collision detected!")
        
        reward -= 0.1
        
        return reward, done, distance
    
    def stop(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


def train_simple_rl(episodes=50, max_steps=200):
    """Train robot using simple Q-learning - FAST VERSION FOR DEMO"""
    
    print("=" * 60)
    print("REINFORCEMENT LEARNING ROBOT NAVIGATION")
    print("=" * 60)
    print("\nInitializing environment and agent...")
    
    try:
        env = RobotNavigationEnv()
        agent = SimpleQLearningAgent()
        
        agent.load()
        
        print(f"\nStarting FAST training for {episodes} episodes (Demo Mode)")
        print("Goal: Navigate from (-4, -4) to (-3.5, 3.5)")
        print("Press Ctrl+C to stop and save progress")
        print("=" * 60)
        
        for episode in range(episodes):
            env.stop()
            rospy.sleep(0.5)  # Faster reset
            
            distance = env.get_distance_to_goal()
            angle = env.get_angle_to_goal()
            state = agent.discretize_state(env.lidar_data, distance, angle)
            
            total_reward = 0
            prev_distance = None
            
            for step in range(max_steps):
                action_idx = agent.choose_action(state)
                action = agent.actions[action_idx]
                env.execute_action(action)
                
                reward, done, distance = env.calculate_reward(prev_distance)
                prev_distance = distance
                total_reward += reward
                
                angle = env.get_angle_to_goal()
                next_state = agent.discretize_state(env.lidar_data, distance, angle)
                
                agent.update(state, action_idx, reward, next_state)
                state = next_state
                
                if done:
                    break
            
            agent.decay_epsilon()
            
            # Print EVERY episode for demo
            print(f"Episode {episode + 1}/{episodes} | "
                  f"Reward: {total_reward:6.1f} | "
                  f"Steps: {step + 1:3d} | "
                  f"Epsilon: {agent.epsilon:.3f} | "
                  f"Distance: {distance:.2f}m")
            
            # Save every 10 episodes
            if (episode + 1) % 10 == 0:
                agent.save()
        
        env.stop()
        agent.save()
        print("\n" + "=" * 60)
        print("Training complete! Q-table saved.")
        print("=" * 60)
        
    except KeyboardInterrupt:
        print("\n\nTraining interrupted by user.")
        env.stop()
        agent.save()
        print("Progress saved!")
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    train_simple_rl(episodes=50)