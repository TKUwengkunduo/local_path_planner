# local_path_planner


## Introduction
`algorithm_local_path_planner` is a ROS 2-based local path planner that utilizes the **Dynamic Window Approach (DWA)** algorithm for robot navigation. By adjusting the parameters in `robot_config.yaml`, users can optimize navigation performance.

---

## Installation & Build
Execute the following commands to set up the package:

```bash
# Clone the repository
git clone https://github.com/TKUwengkunduo/local_path_planner.git
cd local_path_planner

# Build the package
colcon build

# Source the environment
source install/setup.bash
```

---

## Running the Program

### Running ROS 2 Nodes

1. **Run the DWA path planner node:**
    ```bash
    ros2 run algorithm_local_path_planner dwa
    ```
2. **Run the visualization node:**
    ```bash
    ros2 run algorithm_local_path_planner visualizer
    ```
3. **Publish a test goal point:**
    ```bash
    ros2 topic pub /goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"
    ```

### Running the Simulation Program

1. **Modify robot parameters (if needed):**
    ```bash
    nano src/algorithm_local_path_planner/config/robot_config.yaml
    ```
2. **Run the simulation:**
    ```bash
    python3 src/algorithm_local_path_planner/test/simulation_dwa.py
    ```

---

## Parameter Description
The parameters are stored in `config/robot_config.yaml`. Below are key parameters and their descriptions:

| Parameter                | Description                                      | Default |
|--------------------------|--------------------------------------------------|---------|
| `max_linear_velocity`    | Maximum linear speed (m/s)                       | `0.3`   |
| `min_linear_velocity`    | Minimum linear speed (m/s)                       | `0.0`   |
| `max_angular_velocity`   | Maximum angular speed (rad/s)                    | `1.2`   |
| `linear_velocity_resolution` | Linear velocity resolution (m/s)            | `0.1`   |
| `angular_velocity_resolution` | Angular velocity resolution (rad/s)        | `0.1`   |
| `dt`                     | Time interval (s) for simulation                 | `0.1`   |
| `predict_time`           | Prediction window (s)                            | `3.0`   |
| `robot_radius`           | Robot's safety radius (m)                        | `0.5`   |
| `weight_goal`            | Weight for goal distance in cost function        | `1.0`   |
| `weight_velocity`        | Weight for velocity in cost function             | `0.1`   |
| `weight_obstacle`        | Weight for obstacle avoidance in cost function   | `1.0`   |
| `weight_heading`         | Weight for heading error in cost function        | `0.0`   |
| `limit_control_enabled`  | Enable control space limitation (True/False)     | `true`  |
| `limited_linear_velocity`| Max allowed linear velocity when goal is behind  | `0.1`   |

These parameters affect the robot's navigation behavior. Adjust them accordingly based on your robot's hardware.

---

## ROS 2 Nodes
This project contains two main ROS 2 nodes:

### 1. `dwa` (Local Planner)
**Run command:**
```bash
ros2 run algorithm_local_path_planner dwa
```
**Functionality:**
- Receives the current robot state and goal point (`/goal` topic)
- Computes the optimal local trajectory
- Publishes velocity commands (`/cmd_vel` topic) to move the robot

**Subscribed Topics:**
- `/goal` (`geometry_msgs/msg/Point`) - Target position
- `/obstacle` (`std_msgs/msg/Float32MultiArray`) - List of obstacle positions

**Published Topics:**
- `/cmd_vel` (`geometry_msgs/msg/Twist`) - Velocity commands

---

### 2. `visualizer` (Visualization Tool)
**Run command:**
```bash
ros2 run algorithm_local_path_planner visualizer
```
**Functionality:**
- Displays the robot's trajectory
- Visualizes the DWA-generated local paths
- Renders obstacle positions

---

## Testing & Debugging
- **After modifying `robot_config.yaml`, rebuild the package using `colcon build`**.
- If `dwa` is not functioning properly, check whether the `/goal` topic is being published.
- If the robot does not move, verify if `/cmd_vel` has data output.
- Use `rqt_graph` or `ros2 topic echo /topic_name` for debugging.

---

## Contribution & Feedback
If you have any suggestions or bug reports, feel free to submit an issue or pull request!

---

## License
This project is licensed under the MIT License. You are free to use and modify it.

