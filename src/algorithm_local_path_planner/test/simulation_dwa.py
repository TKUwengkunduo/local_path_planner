#!/usr/bin/env python3
"""
進階 DWA 模擬：
使用 matplotlib 模擬一輛車依據更新後的 DWA 演算法從畫面左側移動至右側，
途中避開少量障礙物，起點與目標點分別固定在畫面的左右兩側。

參數設定將從 YAML 檔案 (src/algorithm_local_path_planner/config/robot_config.yaml) 讀取。
"""

import os
import math
import numpy as np
import matplotlib.pyplot as plt
import yaml

# 讀取 YAML 參數設定檔
script_dir = os.path.dirname(os.path.abspath(__file__))
config_file = os.path.join(script_dir, "..", "config", "robot_config.yaml")
try:
    with open(config_file, 'r') as f:
        params = yaml.safe_load(f)
except Exception as e:
    print(f"讀取 YAML 檔案失敗：{e}")
    exit(1)

def motion(state, v, w, dt):
    """
    車輛運動模型 (unicycle model)
    輸入:
      state: [x, y, theta] 當前狀態
      v: 線速度 (m/s)
      w: 角速度 (rad/s)
      dt: 時間間隔 (s)
    輸出:
      新狀態 [x, y, theta]
    """
    x, y, theta = state
    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt
    theta_new = theta + w * dt
    return [x_new, y_new, theta_new]

def predict_trajectory(state, v, w, dt, predict_time):
    """
    根據當前狀態、候選控制命令與預測時間，模擬運動軌跡
    輸出:
      traj: 狀態序列，每個元素為 [x, y, theta]
    """
    traj = []
    temp_state = state.copy()
    t = 0.0
    while t <= predict_time:
        temp_state = motion(temp_state, v, w, dt)
        traj.append(temp_state.copy())
        t += dt
    return traj

def evaluate_trajectory(traj, v, goal, obstacles, params):
    """
    根據四個指標評估軌跡：
      1. 目標距離成本：以軌跡末端與目標的歐式距離 (越近成本越低)
      2. 速度成本：鼓勵較高線速度
      3. 障礙物成本：軌跡上與障礙物最小距離，若小於安全半徑則大幅懲罰
      4. Heading 誤差成本：以平方懲罰最終朝向與目標方向（依當前位置計算）的偏差
    返回總成本 (分數越高越好)
    """
    last_state = traj[-1]
    # 1. 目標距離成本
    goal_dist = math.hypot(goal[0] - last_state[0], goal[1] - last_state[1])
    cost_goal = -params["weight_goal"] * goal_dist

    # 2. 速度成本
    cost_velocity = params["weight_velocity"] * v

    # 3. 障礙物成本
    if not obstacles:
        cost_obstacle = 0.0
    else:
        min_obs_dist = float('inf')
        for state in traj:
            for obs in obstacles:
                dist = math.hypot(obs[0] - state[0], obs[1] - state[1])
                if dist < min_obs_dist:
                    min_obs_dist = dist
        if min_obs_dist <= params["robot_radius"]:
            cost_obstacle = -float('inf')
        else:
            cost_obstacle = params["weight_obstacle"] * min_obs_dist

    # 4. Heading 誤差成本：依據機器人目前狀態與目標間的相對角度來計算
    desired_angle = math.atan2(goal[1] - last_state[1], goal[0] - last_state[0])
    final_heading = last_state[2]
    heading_error = abs(desired_angle - final_heading)
    if heading_error > math.pi:
        heading_error = 2 * math.pi - heading_error
    cost_heading = params["weight_heading"] * (heading_error ** 2)

    total_cost = cost_goal + cost_velocity + cost_obstacle - cost_heading
    return total_cost

def dwa_planning(state, goal, obstacles, params):
    """
    在候選控制空間中搜尋最佳控制命令
    輸出:
      best_control: [v, w]
      best_traj: 與最佳控制對應的預測軌跡
    """
    best_score = -float('inf')
    best_control = [0.0, 0.0]
    best_traj = None

    v_min = params["min_linear_velocity"]
    v_max = params["max_linear_velocity"]
    v_res = params["linear_velocity_resolution"]
    w_max = params["max_angular_velocity"]
    w_res = params["angular_velocity_resolution"]

    for v in np.arange(v_min, v_max + v_res, v_res):
        for w in np.arange(-w_max, w_max + w_res, w_res):
            traj = predict_trajectory(state, v, w, params["dt"], params["predict_time"])
            score = evaluate_trajectory(traj, v, goal, obstacles, params)
            if score > best_score:
                best_score = score
                best_control = [v, w]
                best_traj = traj

    return best_control, best_traj

def simulate_dwa():
    """
    主模擬函式：
      - 初始化車輛起點、目標點及障礙物
      - 利用更新後的 DWA 演算法選擇控制命令並更新車輛狀態
      - 當車輛接近目標 (< 1.0 m) 時，強制停車避免撞上目標
      - 利用 matplotlib 即時更新顯示車輛、軌跡、障礙物與目標點，
        並顯示目前的線速度與角速度
    """
    # 初始狀態：固定於畫面左側，初始朝向正右 (0度)
    state = [0.0, 0.0, 0.0]
    # 目標點：固定於畫面右側 (例如 (10,0))
    goal = [10.0, 0.0]
    # 障礙物：少量障礙物 (可依需求調整)
    obstacles = [(3, 0), (5, -1), (7, 1)]

    # 初始化模擬畫面
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_xlim(-1, 12)
    ax.set_ylim(-5, 5)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("DWA 模擬")
    ax.grid(True)
    # 繪製目標點 (紅色星號)
    ax.plot(goal[0], goal[1], 'r*', markersize=20, label="Goal")
    # 繪製障礙物 (黑色圓形)
    for obs in obstacles:
        obs_circle = plt.Circle(obs, 0.3, color='black')
        ax.add_patch(obs_circle)
    # 用以記錄車輛行走軌跡
    trajectory_history = [state[:2]]
    dt = params["dt"]
    max_steps = 500

    for step in range(max_steps):
        # 當車輛接近目標 (<1.0 m) 時，直接停車避免撞上目標
        if math.hypot(goal[0] - state[0], goal[1] - state[1]) < 1.0:
            print("到達目標,提前結束模擬")
            # 更新圖形顯示最終狀態
            ax.cla()
            ax.set_xlim(-1, 12)
            ax.set_ylim(-5, 5)
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.set_title("DWA 模擬")
            ax.grid(True)
            ax.plot(goal[0], goal[1], 'r*', markersize=20, label="Goal")
            for obs in obstacles:
                obs_circle = plt.Circle(obs, 0.3, color='black')
                ax.add_patch(obs_circle)
            traj_x, traj_y = zip(*trajectory_history)
            ax.plot(traj_x, traj_y, 'b-', linewidth=2, label="Trajectory")
            arrow_length = 0.8
            ax.arrow(state[0], state[1],
                     arrow_length * math.cos(state[2]),
                     arrow_length * math.sin(state[2]),
                     head_width=0.3, head_length=0.3,
                     fc='blue', ec='blue', label="Car")
            ax.legend()
            plt.pause(0.05)
            break

        control, best_traj = dwa_planning(state, goal, obstacles, params)
        v, w = control

        # 更新車輛狀態
        state = motion(state, v, w, dt)
        trajectory_history.append(state[:2])

        # 更新圖形
        ax.cla()
        ax.set_xlim(-1, 12)
        ax.set_ylim(-5, 5)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("DWA 模擬")
        ax.grid(True)
        # 繪製目標點與障礙物
        ax.plot(goal[0], goal[1], 'r*', markersize=20, label="Goal")
        for obs in obstacles:
            obs_circle = plt.Circle(obs, 0.3, color='black')
            ax.add_patch(obs_circle)
        # 繪製行走軌跡
        traj_x, traj_y = zip(*trajectory_history)
        ax.plot(traj_x, traj_y, 'b-', linewidth=2, label="Trajectory")
        # 繪製預測軌跡（虛線）
        if best_traj is not None:
            pred_x = [s[0] for s in best_traj]
            pred_y = [s[1] for s in best_traj]
            ax.plot(pred_x, pred_y, 'c--', linewidth=1, label="Predicted Trajectory")
        # 繪製車輛（以箭頭表示）
        arrow_length = 0.8
        ax.arrow(state[0], state[1],
                 arrow_length * math.cos(state[2]),
                 arrow_length * math.sin(state[2]),
                 head_width=0.3, head_length=0.3,
                 fc='blue', ec='blue', label="Car")
        # 顯示目前的速度與角速度
        ax.text(0.05, 0.95,
                f"v = {v:.2f} m/s\nw = {w:.2f} rad/s",
                transform=ax.transAxes,
                fontsize=12, color='green',
                verticalalignment='top',
                bbox=dict(facecolor='white', alpha=0.5))
        ax.legend()
        plt.pause(0.05)
    else:
        print("到達模擬時間")

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    simulate_dwa()
