# Computer Vision Module

## Overview

The Computer Vision module provides the perception layer for the autonomous vehicle system. It detects traffic signs and other visual signals from a Jetson-connected camera using OpenCV-based preprocessing and two alternative classification pipelines: a histogram-and-shape approach and a SIFT feature-matching approach.

In the full ROS 2 system, the detections published by this module are consumed by downstream control and mission logic. The vision nodes publish the current visual interpretation to `/detected_signal`, where it can be fused with odometry, obstacle detection, and path-following behavior. The module is designed to be lightweight, camera-driven, and easy to swap between detection strategies without changing the rest of the vehicle stack.

## Architecture

```text
								 +----------------------+
								 |  Camera /dev/video0  |
								 +----------+-----------+
												|
					 +-------------------+-------------------+
					 |                                       |
					 v                                       v
+-----------------------------------+   +-----------------------------------+
| Histogram pipeline                 |   | SIFT pipeline                     |
|                                   |   |                                   |
| sign_detector_direct_node.py      |   | sift_node.py                      |
|  - detect_papers()                |   |  - color gate                     |
|  - has_signal_content()           |   |  - SIFT keypoint matching         |
|  - classify_signal()              |   |  - temporal stability filter      |
|  - load_reference_descriptors()    |   |  - publishes /detected_signal     |
|  - publishes /detected_signal     |   |                                   |
+------------------+----------------+   +------------------+----------------+
						 \_______________________   ______________________________/
														 \/
										+---------------------------+
										| Downstream controller /   |
										| mission logic             |
										| subscribes to             |
										| /detected_signal          |
										+---------------------------+
```

## Python Files

| File | Purpose | Node name | Subscribes to | Publishes to | Parameters / config values | Key logic |
| --- | --- | --- | --- | --- | --- | --- |
| `Histogram/sign_vision_core.py` | Shared computer vision utilities for the histogram-based sign detector. It isolates candidate white sign regions, filters sign-like content, builds descriptors, loads reference examples, and compares candidates against known classes. | Not a ROS 2 node | None | None | No ROS parameters. Internal thresholds include white-patch filtering, color-ratio gates, histogram binning, and shape similarity scoring. | Detects vertical white sign-like rectangles, crops the inner region, normalizes color, builds a combined HSV histogram and binary shape mask, then classifies by comparing against reference descriptors. |
| `Histogram/sign_detector_direct_node.py` | ROS 2 camera node that applies the histogram-based pipeline and publishes the detected traffic sign label and confidence. | `sign_detector_direct_node` | None | `/detected_signal` (`std_msgs/String`) | `camera_index`, `publish_topic`, `min_score`, `process_fps`, `stable_frames`, `debug` | Reads frames from the camera, finds candidate signs via `sign_vision_core`, classifies each candidate against stored references, applies a temporal stability filter, and publishes stable detections only. |
| `SIFT/sift_node.py` | ROS 2 camera node that performs feature-based traffic sign recognition using SIFT descriptors and brute-force matching. | `sift_signal_node` | None | `/detected_signal` (`std_msgs/String`) | No ROS parameters. Uses a reference image database loaded from the `fotos` folder under the `integration_test_1` package share directory, plus hard-coded matching thresholds. | Uses a color gate to reject non-sign frames, computes SIFT features for the live frame, compares them against stored descriptor sets, keeps the best-scoring class, and smooths output over multiple frames before publishing. |

## File Details

### `Histogram/sign_vision_core.py`

**Purpose:** Shared computer vision library for the histogram-based detector. It contains the low-level image processing and classification functions used by the ROS 2 node.

**Node name:** Not a ROS 2 node.

**Subscribes to:** None.

**Publishes to:** None.

**Parameters / config values:** No ROS parameters. The module uses hard-coded thresholds for white detection, aspect ratio filtering, color content detection, histogram normalization, and mask similarity.

**Key logic:**

- Detects likely sign candidates by finding tall white rectangles in the frame.
- Crops the interior of the candidate to reduce white borders.
- Applies a simple gray-world white balance to stabilize color.
- Rejects non-sign candidates using color-content thresholds.
- Builds a descriptor from an HSV histogram and a non-white shape mask.
- Loads reference descriptors from class-based folders and compares them using histogram correlation plus IoU mask similarity.

### `Histogram/sign_detector_direct_node.py`

**Purpose:** ROS 2 vision node that captures camera frames, runs the histogram-based detector, and publishes stable traffic sign detections.

**Node name:** `sign_detector_direct_node`

**Subscribes to:** None.

**Publishes to:** `/detected_signal`

**Parameters / config values:**

- `camera_index`: camera device index used by OpenCV.
- `publish_topic`: output topic for detections.
- `min_score`: minimum classification score accepted as a valid label.
- `process_fps`: frame processing rate.
- `stable_frames`: number of repeated classifications required before publishing.
- `debug`: enables additional logging.
- `reference_folder`: derived from the node directory and expected to contain class subfolders with sample images.

**Key logic:**

- Opens the camera and periodically reads frames.
- Detects sign candidates using `detect_papers()`.
- Runs a content check before classification to avoid false positives.
- Classifies each crop with `classify_signal()` against the stored reference descriptors.
- Publishes only after the same label has been observed for several frames.

### `SIFT/sift_node.py`

**Purpose:** ROS 2 vision node that performs traffic sign recognition using SIFT feature matching.

**Node name:** `sift_signal_node`

**Subscribes to:** None.

**Publishes to:** `/detected_signal`

**Parameters / config values:**

- No ROS parameters are declared.
- Uses `integration_test_1` package share directory to locate a `fotos/` database of reference images.
- Uses hard-coded SIFT, ratio-test, and minimum-match thresholds for classification.

**Key logic:**

- Loads reference descriptors from class-organized folders.
- Applies a color gate to quickly reject frames that do not resemble a sign.
- Extracts SIFT keypoints and descriptors from the current frame.
- Matches live descriptors against each reference class with a brute-force matcher and ratio test.
- Chooses the best-scoring class and applies temporal smoothing before publishing.

## Topic Flow

```text
Camera frame
	|
	+--> Histogram/sign_detector_direct_node.py -----> /detected_signal ----+
	|                                                                       |
	+--> SIFT/sift_node.py --------------------------> /detected_signal ----+--> Control / mission logic
```

## Notes For Integration

- Run only one vision detector at a time to avoid conflicting detections on `/detected_signal`.
- The histogram pipeline is easier to tune when signs have a strong white border and clear internal colors.
- The SIFT pipeline is more feature-based and depends on a reference image set that matches the operating environment.
- Both detectors are designed to work alongside the vehicle controller, obstacle avoidance, and navigation nodes used elsewhere in the project.

## Suggested Runtime Setup

1. Start the camera and confirm the device index used by OpenCV.
2. Launch either the histogram detector or the SIFT detector.
3. Subscribe to `/detected_signal` from the controller or mission logic that decides how the vehicle should react.

## Maintenance Tips

- Keep reference images organized by class name so classification stays consistent.
- Tune thresholds in `sign_vision_core.py` if lighting, camera exposure, or sign styles change.
- Update the README whenever new Python files are added to this folder so the documentation stays aligned with the code.
