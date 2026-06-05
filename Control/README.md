# Control Module

## Overview

The Control module turns perception and localization information into motion commands for the vehicle. It contains a Lyapunov-style controller for goal seeking and a higher-level integrated controller that arbitrates between obstacle information, traffic-sign interpretation, and navigation behavior.

Within the full ROS 2 system, this module is the direct bridge to actuation. It subscribes to odometry, obstacle state, and detection topics, then publishes `/cmd_vel` commands that drive the robot forward, stop it for a sign, or steer it toward the target while respecting the current operating context.

## Architecture

```text
      /odom ----------------------+
      /obstacle_info ------------+ |
      /detected_signal ---------+| |
                                v| v
                    +---------------------------+
                    | controller_integration... |
                    |  - obstacle priority      |
                    |  - vision stop priority   |
                    |  - Lyapunov fallback      |
                    +-------------+-------------+
                                  |
                                  v
                               /cmd_vel

      /odom ----------------------+
                                v
                    +---------------------------+
                    | lyapunov_node.py          |
                    |  - goal-seeking control   |
                    +-------------+-------------+
                                  |
                                  v
                               /cmd_vel
```

## Python Files

### `controller_integration_vision_avoidance.py`

**Purpose:** Integrated controller that combines odometry, obstacle detection, and sign detection to decide the final robot motion command.

**Node name:** `smart_controller`

**Subscribes to:** `/odom`, `/obstacle_info`, `/detected_signal`

**Publishes to:** `/cmd_vel`

**Parameters / config values:** Hard-coded goal position, vehicle geometry, controller gains, and goal tolerance are defined in the script.

**Key logic:** Gives obstacle avoidance the highest priority, stops on a `stop` sign, and otherwise applies Lyapunov-based navigation control toward the target pose.

### `lyapunov_node.py`

**Purpose:** Standalone Lyapunov controller for goal-seeking motion.

**Node name:** `lyapunov_controller`

**Subscribes to:** `/odom`

**Publishes to:** `/cmd_vel`

**Parameters / config values:** Hard-coded goal position, vehicle dimensions, gains, and goal tolerance are embedded in the script.

**Key logic:** Computes position and heading error relative to a fixed goal, derives a control law using a Lyapunov-inspired formulation, applies saturation, and publishes a smooth velocity command.

## Topic Flow

```text
/odom -------------> controller_integration_vision_avoidance.py ----+
/obstacle_info ---->                                             |   |
/detected_signal -->                                             v   v
                                                              /cmd_vel

/odom -------------> lyapunov_node.py ---------------------------> /cmd_vel
```

## Runtime Notes

- Do not run `lyapunov_node.py` together with `controller_integration_vision_avoidance.py` if both are intended to command the robot, because they publish to the same `/cmd_vel` topic.
- `controller_integration_vision_avoidance.py` expects obstacle information in the string format produced by the obstacle module and detections in the `label,score` format used by the vision module.
- Both nodes assume the robot pose information from `/odom` is available and reasonably stable.
