# Obstacle Avoidance Module

## Overview

The Obstacle Avoidance module processes LIDAR data to detect nearby obstacles and protect the vehicle from collisions. It provides two operating modes: one node publishes a compact obstacle summary for other modules to consume, while the other node directly commands the robot to avoid obstacles with a simple state machine.

In the full ROS 2 system, this module can either act as a perception source for the controller or as an active avoidance layer that writes directly to `/cmd_vel`. That flexibility makes it useful both for integrated autonomy and for standalone safety behavior during testing.

## Architecture

```text
                         +--------------+
                         |    /scan     |
                         +------+-------+
                                |
                                v
         +----------------------------------------+
         | avoidance_publisher_obstacle.py        |
         |  - finds closest valid obstacle        |
         |  - publishes /obstacle_info            |
         +------------------+---------------------+
                            |
                            v
                     /obstacle_info

                         +--------------+
                         |    /scan     |
                         +------+-------+
                                |
                                v
         +----------------------------------------+
         | avoidance_node.py                      |
         |  - state machine avoidance logic       |
         |  - publishes /cmd_vel directly         |
         +------------------+---------------------+
                            |
                            v
                         /cmd_vel
```

## Python Files

### `avoidance_publisher_obstacle.py`

**Purpose:** Lightweight LIDAR processing node that detects the nearest obstacle and publishes a compact status string.

**Node name:** `obstacle_avoidance`

**Subscribes to:** `/scan`

**Publishes to:** `/obstacle_info`

**Parameters / config values:** `safe_distance`

**Key logic:** Iterates through laser ranges, ignores invalid readings, tracks the closest obstacle, and publishes either `"1,angle,distance"` when an obstacle is too close or `"0,0,0"` when the path is clear.

### `avoidance_node.py`

**Purpose:** Direct obstacle avoidance controller that publishes robot motion commands without relying on another high-level controller.

**Node name:** `obstacle_avoidance`

**Subscribes to:** `/scan`

**Publishes to:** `/cmd_vel`

**Parameters / config values:** `safe_distance`, `critical_distance`, plus the internal state-machine timing and speed constants in the script.

**Key logic:** Uses a state machine with forward, turning, and backing-up phases. It detects the closest obstacle, chooses a turn direction, backs up only when safe, and accepts keyboard input to stop or resume motion.

## Topic Flow

```text
/scan --> avoidance_publisher_obstacle.py --> /obstacle_info --> Control module

/scan --> avoidance_node.py ----------------------------------> /cmd_vel
```

## Runtime Notes

- Use `avoidance_publisher_obstacle.py` when obstacle state should be consumed by a higher-level controller.
- Use `avoidance_node.py` when you want the obstacle logic itself to command the robot.
- Do not run `avoidance_node.py` together with another node that also controls `/cmd_vel` unless you intentionally want competing velocity publishers.
