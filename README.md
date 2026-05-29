# NOTE: First draft.

# Implementation of Intelligent Robotics (Gpo 501)

The challenge consists of developing an autonomous system capable of locating objects within a warehouse using a variety of advanced technologies. Students must integrate control algorithms, computer vision, navigation, and artificial intelligence (AI) to develop a robot capable of performing object localization tasks within a warehouse with known geometry. This project is built on ROS2 and targets embedded platforms (Jetson Nano or similar).

---

**Repository overview**

- `Computer Vision/` — Vision nodes and helpers for sign detection.
	- `Histogram/` — Lightweight histogram+shape detector (`sign_vision_core.py`, `sign_detector_direct_node.py`).
	- `SIFT/` — SIFT-based detector (`sift_node.py`).
- `Control/` — Controllers and the integration node.
	- `lyapunov_node.py` — Standalone Lyapunov navigation controller (publishes to `/cmd_vel`).
	- `controller_integration_vision_avoidance.py` — `SmartController` that fuses odometry, obstacle info and vision to publish `/cmd_vel`.
- `Navigation/` — Pose conversion utilities.
	- `pose_republisher.py` — Republishes `/amcl_pose` → `/robot_pose`.
- `Obstacle Avoidance/` — LIDAR processing and avoidance logic.
	- `avoidance_publisher_obstacle.py` — LaserScan → `/obstacle_info` (decoupled obstacle detector).
	- `avoidance_node.py` — State-machine obstacle avoider that publishes `/cmd_vel` directly.

**Key ROS topics**

- `/scan` — sensor_msgs/LaserScan from LIDAR
- `/obstacle_info` — std_msgs/String ("1,angle,distance" or "0,0,0")
- `/detected_signal` — std_msgs/String ("label,score")
- `/odom` — nav_msgs/Odometry
- `/cmd_vel` — geometry_msgs/Twist (robot velocity commands)

**Recommended runtime configuration**

Run one vision node (either the histogram-based `sign_detector_direct_node.py` or `sift_node.py`), the `avoidance_publisher_obstacle.py`, the `pose_republisher.py` (if using AMCL), and the `controller_integration_vision_avoidance.py` as the main decision hub. Do NOT run `lyapunov_node.py` or `avoidance_node.py` together with the `SmartController` to avoid conflicting publishers on `/cmd_vel`.

**Quick start (ROS2)**

1. Ensure ROS2 and dependencies (`opencv-python`, `numpy`) are installed on your platform.
2. Build and source your ROS2 workspace as usual.
3. Start nodes (example commands depend on your package structure):

```bash
# Start obstacle detector
ros2 run obstacle_avoidance avoidance_publisher_obstacle

# Start vision (pick one)
ros2 run computer_vision_histogram sign_detector_direct_node
# or
ros2 run computer_vision_sift sift_node

# Start controller
ros2 run control controller_integration_vision_avoidance
```

Adjust package and executable names to your ROS2 package layout or use a launch file.

**Development notes & suggestions**

- Choose one vision backend (histogram or SIFT) to avoid mixed detections on `/detected_signal`.
- `SmartController` priority: obstacle (highest) → vision (stop) → Lyapunov navigation (default).
- The histogram node supports a `debug` parameter for additional logging; SIFT node displays a visualization window.

