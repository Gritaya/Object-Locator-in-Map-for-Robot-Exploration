# Object Locator for Semantic Mapping in ROS2

## Overview

This project implements a ROS2 object localization node for semantic mapping on a TurtleBot.
<img width="1446" height="1084" alt="image" src="https://github.com/user-attachments/assets/2f8f985e-f1c1-4447-9e3d-78c280348faa" />

Traditional autonomous navigation systems generate geometric maps that represent free space and obstacles but do not provide information about the identity or location of specific objects. This project extends conventional mapping by estimating the global positions of detected objects and visualizing them within the robot's map frame.

Using an onboard RPi4 camera, the system detects ArUco markers, estimates their 3D pose through OpenCV's solvePnP() algorithm, and transforms the resulting coordinates from the camera frame to the global map frame using ROS 2 TF. The localized objects are then published as visualization markers for display in RViz.

The node is designed to integrate with SLAM and autonomous exploration systems, enabling the creation of semantically enriched maps of previously unknown environments

## Project Context

This node was developed as part of a larger autonomous mobile robot system capable of exploring unknown indoor environments and constructing semantically enriched maps.

The complete system includes:

* ROS2
* TurtleBot platform
* SLAM for localization and mapping
* Frontier-based autonomous exploration
* ArUco marker detection
* Semantic object localization
* RViz visualization

## Method

### 1. Marker Detection

The robot camera detects ArUco markers in the environment using OpenCV.

### 2. Pose Estimation

The marker pose relative to the camera is estimated using OpenCV's `solvePnP()` function.

Output:

```text
Camera Frame
x = horizontal displacement
z = forward depth
```

### 3. Coordinate Transformation

ROS 2 TF transformations are used to convert object coordinates from the camera frame into the global map frame.

Transformation chain:

## TF Tree

```text
map (global SLAM frame)
 └── odom (local odometry frame)
      └── base_link (robot center)
           ├─ Dynamic TF from robot localization
           │
           └── camera_link (physical camera mount)
               Static TF (baselink -> camera link):
               Translation = (0.05, 0.00, 0.15) m
               Rotation    = (0, 0, 0) rad
               │
               └── camera_optical_frame (OpenCV optical frame)
                   Static TF (camera_link -> camera_optical_frame):
                   Translation = (0.00, 0.00, 0.00) m
                   Rotation    = (-1.5708, 0, -1.5708) rad
```
### Static Transform Publishers

#### base_link → camera_link

```bash
ros2 run tf2_ros static_transform_publisher \
0.05 0.0 0.15 \
0 0 0 \
base_link camera_link
```

#### camera_link → camera_optical_frame

```bash
ros2 run tf2_ros static_transform_publisher \
0 0 0 \
-1.5708 0 -1.5708 \
camera_link camera_optical_frame
```

### 4. Map Projection

The system projects detected marker positions onto a 2D map.

```text
Camera coordinates:
(x, z)

Map coordinates:
(x, y)
```

where:

* Camera depth (z) becomes map forward distance
* Camera horizontal displacement (x) becomes map lateral position

### 5. Visualization

Localized objects are published as ROS2 visualization markers and displayed in RViz.

---

## System Architecture

```text
Camera
   │
   ▼
ArUco Detection
   │
   ▼
solvePnP Pose Estimation
   │
   ▼
TF Frame Transformation
   │
   ▼
Object Localization
   │
   ▼
RViz Visualization
```

---

## Package Structure

```text
object_locator/
├── object_locator/
│   └── object_locator_node.py
├── setup.py
├── package.xml
└── README.md
```
## RViz Visualization

Launch RViz and add:

* Marker
* TF
* Map

Detected objects will appear at their estimated global positions.
<img width="1518" height="1084" alt="image" src="https://github.com/user-attachments/assets/8c400aa8-4abc-4127-a3be-1d4bde655c0f" />

---

## Results

The node successfully:
* Detects ArUco markers in real time
* Estimates marker position using solvePnP
* Converts coordinates into the map frame using ROS2 TF
* Displays semantic object locations in RViz

## Future Work
* Integration with object detection models (YOLO, MobileNet, etc.) for object identification
* Semantic occupancy grid generation
* Multi-object tracking

## Authors

Developed for autonomous semantic mapping research using TurtleBot and ROS2.
