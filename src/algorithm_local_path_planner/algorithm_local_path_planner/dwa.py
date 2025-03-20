#!/usr/bin/env python3
"""
This program is an example implementation of a DWA node based on ROS2.
All adjustable parameters are provided by a YAML file (robot_config.yaml) in the config folder.
The goal is specified in the robot's relative coordinate system, and planning is simulated using [0,0,0] as the starting state.
"""

import os
import math
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32MultiArray

# Get package share directory to load the YAML config file
from ament_index_python.packages import get_package_share_directory


class DWAPlanner(Node):
    def __init__(self):
        # Initialize node
        super().__init__('dwa_planner')

        ###############################################################
        # Read YAML configuration file
        ###############################################################
        pkg_share = get_package_share_directory('algorithm_local_path_planner')
        config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
        self.get_logger().info(f"Reading config file: {config_file}")
        try:
            with open(config_file, 'r') as f:
                config_params = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML file: {e}")
            rclpy.shutdown()
            return

        # Load parameters from YAML
        self.max_v = config_params.get('max_linear_velocity')
        self.min_v = config_params.get('min_linear_velocity')
        self.max_w = config_params.get('max_angular_velocity')
        self.v_resolution = config_params.get('linear_velocity_resolution')
        self.w_resolution = config_params.get('angular_velocity_resolution')
        self.dt = config_params.get('dt')
        self.predict_time = config_params.get('predict_time')
        self.robot_radius = config_params.get('robot_radius')
        self.weight_goal = config_params.get('weight_goal')
        self.weight_velocity = config_params.get('weight_velocity')
        self.weight_obstacle = config_params.get('weight_obstacle')
        self.weight_heading = config_params.get('weight_heading', 1.0)

        # 新增：是否啟用限制控制空間，及限制的線速度
        self.limit_control_enabled = config_params.get('limit_control_enabled', False)
        self.limited_linear_velocity = config_params.get('limited_linear_velocity', self.max_v)

        ###############################################################
        # Initialize internal variables
        ###############################################################
        # Received goal (relative coordinates) as [x, y]
        self.goal = None
        # Obstacles list, format: [[x, y], [x, y], ...]
        self.obstacles = []
        # Since planning is based on relative coordinates, always assume the robot is at [0,0,0]
        self.current_state = [0.0, 0.0, 0.0]

        ###############################################################
        # Set up ROS2 publishers and subscribers
        ###############################################################
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.goal_sub = self.create_subscription(Point, 'goal', self.goal_callback, 1)
        self.obs_sub = self.create_subscription(Float32MultiArray, 'obstacle', self.obstacle_callback, 1)

        # Timer to run DWA planning every dt seconds
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info("DWA Planner node (relative coordinate mode) started successfully.")

    def goal_callback(self, msg: Point):
        """
        Process /goal topic message to update the goal (relative coordinates).
        """
        self.goal = [msg.x, msg.y]
        self.get_logger().info(f"Received new goal (relative coordinates): {self.goal}")

    def obstacle_callback(self, msg: Float32MultiArray):
        """
        Process /obstacle topic message to update obstacles.
        Data format is assumed to be a 1D list where every two numbers represent an obstacle's x, y.
        """
        data = msg.data
        self.obstacles = []
        if len(data) % 2 == 0:
            for i in range(0, len(data), 2):
                self.obstacles.append([data[i], data[i+1]])
        else:
            self.get_logger().error("Received obstacle data length error; the number of elements should be even.")

    def timer_callback(self):
        """
        Every dt seconds, using the assumption that the robot is at [0,0,0] in its relative frame,
        choose the best control command using DWA based on the current goal and obstacles,
        then publish the command to /cmd_vel.
        """
        best_u = [0.0, 0.0]

        if self.goal is None :
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().error(f"no target")
            return
        else:
            if (math.sqrt(self.goal[0]**2 + self.goal[1]**2)<1.0):
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().info(f"reach target point")
                return

        # Determine candidate maximum linear velocity
        # If the goal is behind (i.e. goal[0] < 0) and limit control is enabled, use limited_linear_velocity.
        if self.goal[0] < 0 and self.limit_control_enabled:
            v_max_sample = min(self.max_v, self.limited_linear_velocity)
        else:
            v_max_sample = self.max_v

        # Planning starting point (always assume [0,0,0] in relative coordinate system)
        initial_state = [0.0, 0.0, 0.0]

        # best_u = [0.0, 0.0]         # Store best control command [v, w]
        best_score = -float('inf')  # Initialize best score

        # Sample the control space: use v_max_sample for linear velocity sampling.
        for v in np.arange(self.min_v, v_max_sample + self.v_resolution, self.v_resolution):
            for w in np.arange(-self.max_w, self.max_w + self.w_resolution, self.w_resolution):
                traj = self.predict_trajectory(initial_state, v, w)
                score = self.evaluate_trajectory(traj, v)
                if score > best_score:
                    best_score = score
                    best_u = [v, w]

        twist = Twist()
        twist.linear.x = best_u[0]
        twist.angular.z = best_u[1]
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"Publishing cmd_vel: linear velocity = {best_u[0]:.2f} m/s, angular velocity = {best_u[1]:.2f} rad/s")

    def predict_trajectory(self, state, v, w):
        """
        Simulate future trajectory based on the initial state, candidate control command, and prediction time.
        Inputs:
            state: initial state [x, y, theta] (fixed as [0,0,0])
            v: candidate linear velocity
            w: candidate angular velocity
        Output:
            traj: simulated trajectory (list of states)
        """
        traj = []
        temp_state = state.copy()
        time = 0.0
        while time <= self.predict_time:
            temp_state = self.motion(temp_state, v, w, self.dt)
            traj.append(temp_state.copy())
            time += self.dt
        return traj

    def motion(self, state, v, w, dt):
        """
        Robot motion model (unicycle model).
        Inputs:
            state: current state [x, y, theta]
            v: linear velocity (m/s)
            w: angular velocity (rad/s)
            dt: time interval (s)
        Output:
            new state [x, y, theta]
        """
        x, y, theta = state
        x_new = x + v * math.cos(theta) * dt
        y_new = y + v * math.sin(theta) * dt
        theta_new = theta + w * dt
        return [x_new, y_new, theta_new]

    def evaluate_trajectory(self, traj, v):
        """
        Evaluate trajectory quality considering goal distance, velocity, obstacle cost, and heading error.
        Inputs:
            traj: predicted trajectory (list of states)
            v: candidate linear velocity
        Output:
            total score (higher means better trajectory)
        """
        # 1. Goal distance cost: distance from trajectory end to goal (closer is better)
        last_state = traj[-1]
        goal_dist = math.hypot(self.goal[0] - last_state[0], self.goal[1] - last_state[1])
        cost_goal = -self.weight_goal * goal_dist

        # 2. Velocity cost: encourage higher linear velocity
        cost_velocity = self.weight_velocity * v

        # 3. Obstacle cost: penalize trajectories that come too close to obstacles
        if not self.obstacles:
            cost_obstacle = 0.0
        else:
            min_obs_dist = float('inf')
            for state in traj:
                for obs in self.obstacles:
                    dist = math.hypot(obs[0] - state[0], obs[1] - state[1])
                    if dist < min_obs_dist:
                        min_obs_dist = dist
            if min_obs_dist <= self.robot_radius:
                cost_obstacle = -float('inf')
            else:
                cost_obstacle = self.weight_obstacle * min_obs_dist

        # 4. Heading error cost: penalize mismatch between final heading and desired heading toward goal
        desired_angle = math.atan2(self.goal[1], self.goal[0])
        final_heading = last_state[2]
        heading_error = abs(desired_angle - final_heading)
        if heading_error > math.pi:
            heading_error = 2 * math.pi - heading_error
        # Using square error for stronger penalty on large deviations
        cost_heading = self.weight_heading * (heading_error ** 2)

        total_cost = cost_goal + cost_velocity + cost_obstacle - cost_heading
        return total_cost


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
