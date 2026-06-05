# Navigation Module

## Overview

The Navigation module handles localization, global planning, path following, and short recovery maneuvers for the autonomous vehicle. It works with the mapped environment to estimate the robot pose, compute a collision-free path, and convert that path into motion commands that can drive the platform to a goal.

In the full ROS 2 system, this module sits between perception and actuation. It consumes the occupancy grid map, scan data, and pose estimates, then publishes a planned path or direct velocity commands depending on the active node. The overall result is a navigation stack that can localize the vehicle, plan a route, follow that route, and recover with a short reverse maneuver when needed.

## Architecture

```text
          +-------------------+
          |   /initialpose    |
          +---------+---------+
                    |
                    v
      +-------------------------------+
      | initial_pose_publisher.py     |
      +---------------+---------------+
                      |
                      v
      +-------------------------------+        +----------------------+
      | scan_match_localizer_node.py  |-------> |      /robot_pose     |
      +-------------------------------+        +----------+-----------+
                                                          |
                                                          v
      +-------------------------------+        +----------------------+
      | astar_planner_node.py         |<------- |       /map          |
      +---------------+---------------+        +----------+-----------+
                      |                                  |
                      v                                  |
               /planned_path                             |
                      |                                  |
                      v                                  |
      +-------------------------------+                   |
      | path_follower_node.py         |-------------------+
      +---------------+---------------+
                      |
                      v
                  /cmd_vel

      +-------------------------------+
      | reverse_escape_node.py        |
      +---------------+---------------+
                      |
                      v
                  /cmd_vel

      +-------------------------------+
      | mission_runner.py             |
      | (orchestrates the whole flow) |
      +-------------------------------+
```

## Python Files

### `astar_planner_node.py`

**Purpose:** Global planner that computes a collision-free path over the occupancy grid using A* search.

**Node name:** `astar_planner`

**Subscribes to:** `/map`, `/robot_pose`

**Publishes to:** `/planned_path`

**Parameters / config values:** `goal_x`, `goal_y`, `inflation_radius`, `unknown_as_obstacle`, `replan_period`, `plan_once`, `waypoint_stride`, `allow_start_goal_recovery`, `recovery_radius_cells`

**Key logic:** Inflates occupied cells, converts start and goal from world coordinates into grid cells, optionally recovers from blocked start/goal cells, runs A* over 8-connected neighbors, and publishes a sampled `nav_msgs/Path`.

### `scan_match_localizer_node.py`

**Purpose:** Scan-matching localizer that estimates and republishes the robot pose in the map frame.

**Node name:** `scan_match_localizer`

**Subscribes to:** `/map`, `/scan`, `/initialpose`

**Publishes to:** `/robot_pose`, `/scan_match_score`

**Parameters / config values:** `initial_x`, `initial_y`, `initial_yaw`, `map_frame`, `base_frame`, `publish_rate`, `process_rate_limit`, `laser_min_range`, `laser_max_range`, `max_beams`, `search_xy_range`, `search_yaw_range_deg`, `coarse_xy_step`, `coarse_yaw_step_deg`, `fine_xy_range`, `fine_yaw_range_deg`, `fine_xy_step`, `fine_yaw_step_deg`, `occupied_threshold`, `max_score_distance`, `score_sigma`, `pose_alpha_xy`, `pose_alpha_yaw`, `min_accept_score`

**Key logic:** Builds a distance field from the map, extracts a reduced set of laser points, searches a coarse-to-fine pose neighborhood, scores candidate poses against the map, and publishes a filtered pose estimate.

### `path_follower_node.py`

**Purpose:** Pure-pursuit style path follower that turns a planned path into robot velocity commands.

**Node name:** `path_follower`

**Subscribes to:** `/robot_pose`, `/planned_path`, `/detected_signal`

**Publishes to:** `/cmd_vel`

**Parameters / config values:** `lookahead_distance`, `final_goal_tolerance`, `max_linear`, `min_linear`, `max_angular`, `angular_bias`, `pose_timeout`, `max_search_backwards`

**Key logic:** Tracks the closest path waypoint, selects a lookahead target, computes curvature from the target in the robot frame, slows down for large heading errors, and stops when the goal is reached or a stop signal is received.

### `reverse_escape_node.py`

**Purpose:** Simple timed reverse maneuver used after a mission segment or when the vehicle needs to back out of a tight area.

**Node name:** `reverse_escape_node`

**Subscribes to:** None.

**Publishes to:** `/cmd_vel`

**Parameters / config values:** `reverse_speed`, `angular_z`, `duration`, `publish_rate`

**Key logic:** Publishes a reverse velocity for a fixed duration, optionally applying a turn rate, and then stops the robot cleanly.

### `initial_pose_publisher.py`

**Purpose:** Utility node that publishes the initial pose multiple times so localization can lock onto the map.

**Node name:** `initial_pose_publisher`

**Subscribes to:** None.

**Publishes to:** `/initialpose`

**Parameters / config values:** `x`, `y`, `yaw`, `publish_count`, `start_delay`, `interval`

**Key logic:** Waits for a startup delay, then publishes a `PoseWithCovarianceStamped` several times with map-frame coordinates and a covariance suitable for initialization.

### `mission_runner.py`

**Purpose:** Standalone mission supervisor that launches localization, planning, path following, reverse escape, and cleanup steps in sequence.

**Node name:** Not a ROS 2 node.

**Subscribes to:** None directly; it launches and monitors external ROS 2 processes.

**Publishes to:** None directly.

**Parameters / config values:** Mission setup values are hard-coded in the script, including initial pose, inflation radius, path follower gains, reverse maneuver parameters, and goal coordinates.

**Key logic:** Spawns ROS 2 commands as subprocesses, waits for success patterns in stdout, stops processes safely on failure or completion, and sequences the full navigation mission.

## Topic Flow

```text
/initialpose --> initial_pose_publisher.py --> /initialpose
                     |
                     v
               scan_match_localizer_node.py --> /robot_pose
                     |
                     v
/map --> astar_planner_node.py --> /planned_path --> path_follower_node.py --> /cmd_vel
                                                         |
                                                         +--> /detected_signal

reverse_escape_node.py ----------------------------------> /cmd_vel
```

## Runtime Notes

- `scan_match_localizer_node.py` should be started before planning so `/robot_pose` is available.
- `path_follower_node.py` expects a valid path on `/planned_path` and a current pose on `/robot_pose`.
- `reverse_escape_node.py` and `path_follower_node.py` both publish to `/cmd_vel`, so they should not be commanded at the same time.
- `mission_runner.py` is useful when you want to execute the full navigation sequence from one script.
