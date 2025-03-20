#!/usr/bin/env python3
"""
此程式為一個 ROS2 節點，用以視覺化機器人的目標、控制指令與障礙物資訊。
在此示範如何利用目前的線速度 (v) 與角速度 (w)，推算未來 T 秒後機器人大致前往的方向，
並以一支箭頭表示出來（即「角速度方向的箭頭」）。
"""

import os
import math
import yaml
import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')

        # 讀取 robot_config.yaml
        pkg_share = get_package_share_directory('algorithm_local_path_planner')
        config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
        self.get_logger().info(f"Reading config file: {config_file}")
        try:
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to read config file: {e}")
            rclpy.shutdown()
            return

        # 機器人安全半徑 (畫藍色圓)
        self.robot_radius = self.config.get('robot_radius', 0.5)

        # 內部變數
        self.goal = None            # 目標點 (x, y)
        self.cmd_vel = None         # 控制指令 Twist
        self.obstacles = []         # 障礙物清單 [(x, y), ...]

        # 訂閱 topic
        self.create_subscription(Point, 'goal', self.goal_callback, 1)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(Float32MultiArray, 'obstacle', self.obstacle_callback, 1)

        # 建立 matplotlib 視覺化
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_title("Robot Visualization")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)

        # 畫出機器人 (藍色虛線圓)
        self.robot_circle = plt.Circle((0, 0), self.robot_radius, color='blue',
                                       fill=False, linestyle='--', label="Robot")
        self.ax.add_patch(self.robot_circle)

        # 其他圖形物件
        self.goal_plot, = self.ax.plot([], [], 'r*', markersize=15, label="Goal")
        self.obstacle_plot = self.ax.scatter([], [], c='black', marker='o', label="Obstacles")

        self.ax.legend()
        self.ax.grid(True)

        # 每 0.1 秒更新一次
        self.timer = self.create_timer(0.1, self.timer_callback)

    def goal_callback(self, msg: Point):
        """更新目標點"""
        self.goal = (msg.x, msg.y)
        self.get_logger().info(f"Received goal: {self.goal}")

    def cmd_vel_callback(self, msg: Twist):
        """更新控制指令"""
        self.cmd_vel = msg

    def obstacle_callback(self, msg: Float32MultiArray):
        """更新障礙物"""
        data = msg.data
        if len(data) % 2 == 0:
            obs = []
            for i in range(0, len(data), 2):
                obs.append((data[i], data[i+1]))
            self.obstacles = obs
        else:
            self.get_logger().error("Obstacle data length error; must be even.")

    def timer_callback(self):
        """更新圖形介面"""
        self.ax.cla()
        self.ax.set_title("Robot Visualization")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.grid(True)

        # 畫機器人藍色圓
        self.robot_circle = plt.Circle((0, 0), self.robot_radius, color='blue',
                                       fill=False, linestyle='--', label="Robot")
        self.ax.add_patch(self.robot_circle)

        # 如果有目標
        if self.goal is not None:
            gx, gy = self.goal
            self.ax.plot(gx, gy, 'r*', markersize=15, label="Goal")

        # 如果有障礙物
        if self.obstacles:
            x_obs, y_obs = zip(*self.obstacles)
            self.ax.scatter(x_obs, y_obs, c='black', marker='o', label="Obstacles")

        # 原本若要顯示 cmd_vel 箭頭，也可以留著 (此為線速度箭頭)
        if self.cmd_vel is not None:
            self.ax.arrow(0, 0,
                          self.cmd_vel.linear.x, 0,
                          head_width=0.2, head_length=0.2,
                          fc='cyan', ec='cyan', label="Cmd_vel (linear.x)")

            self.ax.text(0, -0.5, f"Angular: {self.cmd_vel.angular.z:.2f} rad/s",
                         color='cyan', fontsize=10)

            # ==========================
            # 角速度方向箭頭 (關鍵新增)
            # ==========================
            v = self.cmd_vel.linear.x
            w = self.cmd_vel.angular.z

            # 這裡我們用 T=1 秒，若你想用 predict_time，可自行改成 T = self.config.get('predict_time', 1.0)
            T = 1.0
            if abs(w) < 1e-6:
                # w ~ 0，代表直走
                x_end = v * T
                y_end = 0.0
            else:
                # 無輪模型 (unicycle) 封閉解
                x_end = (v / w) * math.sin(w * T)
                y_end = (v / w) * (1 - math.cos(w * T))

            # 畫出「角速度方向箭頭」
            self.ax.arrow(0, 0, x_end, y_end,
                          head_width=0.2, head_length=0.2,
                          fc='green', ec='green', label="Angle-based direction")

        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Visualizer node shutting down...")
    node.destroy_node()
    rclpy.shutdown()
    plt.close(node.fig)


if __name__ == '__main__':
    main()
