# Design and Implementation of an Autonomous Vehicle Using ROS 2

**Course:** Implementation of Intelligent Robotics — TE3002B  
**Institution:** Tecnológico de Monterrey, Campus Guadalajara  

**Authors:**  
Elizabeth Jáuregui Zárate, Gonzalo Flores García, Ana María Rodríguez Peña, Vanessa Cerda Carrillo, Sofia Knutas

---

This repository contains ROS 2-compatible Python nodes and helpers developed for an autonomous Ackermann-steered vehicle. The code integrates computer vision, obstacle avoidance, path planning, and mission orchestration. The nodes are runnable either via `ros2 run` (when packaged) or directly with `python3` after sourcing your ROS 2 environment.

---

**Module documentation**

- [Computer Vision/README.md](Computer%20Vision/README.md)
- [Navigation/README.md](Navigation/README.md)
- [Control/README.md](Control/README.md)
- [Obstacle Avoidance/README.md](Obstacle%20Avoidance/README.md)

---

**Repository overview (current)**

- `Computer Vision/`
	- `Histogram/` — histogram and shape based detector: `sign_vision_core.py`, `sign_detector_direct_node.py`.
	- `SIFT/` — SIFT-based detector: `sift_node.py` (publishes `/detected_signal`).
- `Control/`
	- `controller_integration_vision_avoidance.py` — `SmartController` that subscribes to `/odom`, `/obstacle_info` and `/detected_signal`, publishes `/cmd_vel`.
	- `lyapunov_node.py` — Lyapunov-style controller / path follower variants.
- `Navigation/`
	- planner and helper nodes: `astar_planner_node.py`, `path_follower_node.py`, `mission_runner.py`, `scan_match_localizer_node.py`, `reverse_escape_node.py`, `initial_pose_publisher.py`.
	- `maps/` — example maps (`mapa_pista.pgm`, `mapa_pista.yaml`).
- `Functional_Prototype/` — higher-level orchestrator and support nodes
	- `src/mission_orchestrator_node.py` — mission orchestrator that starts planners, followers and manages mission state.
	- `src/map_republisher_node.py` — republishes an occupancy grid for planning.
	- `config/` — example mission and signal configs (`mission_config.json`, `signal_config.json`).
- `Obstacle Avoidance/`
	- `avoidance_publisher_obstacle.py` — LaserScan → `/obstacle_info` (sends `"1,angle,distance"` when obstacle present, `"0,0,0"` otherwise).

Other folders: `integration_test_2/`, `integration_test_4/` contain packaged test harnesses and launch helpers for experiments.

**Key ROS topics**

- `/scan` — `sensor_msgs/LaserScan` (LIDAR input)
- `/obstacle_info` — `std_msgs/String` (`"1,angle,distance"` or `"0,0,0"`)
- `/detected_signal` — `std_msgs/String` (`"label,score"`)
- `/odom` — `nav_msgs/Odometry`
- `/cmd_vel` — `geometry_msgs/Twist` (velocity command)
- `/mission_state`, `/mission_event` — mission orchestration state/events (strings)

**Prerequisites**

- ROS2 (Foxy/Galactic/Rolling or similar) installed and sourced.
- Python dependencies: `opencv-python`, `numpy`, `rclpy` (installed as part of ROS2 Python packages), `ament_index_python` for some nodes.

Install Python dependencies for local testing (example):

```bash
# From your system Python/venv
pip3 install opencv-python numpy
```

Ensure you source your ROS2 setup before running nodes:

```bash
source /opt/ros/<distro>/setup.bash
```

**Recommended runtime configuration**

- Typical minimal set: one vision node (Histogram or SIFT) + `Obstacle Avoidance/avoidance_publisher_obstacle.py` + `Functional_Prototype/src/map_republisher_node.py` (if using AMCL) + control/orchestrator (`Control/controller_integration_vision_avoidance.py` or `Functional_Prototype/src/mission_orchestrator_node.py`).
- Avoid running multiple nodes that publish to `/cmd_vel` simultaneously (for example do not run `lyapunov_node.py` or `avoidance_node.py` together with `SmartController`).



