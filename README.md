# Autonomous Mobile Systems Coursework

This repository contains a curated set of projects developed for the
**Autonomous Mobile Systems** course at the University of Wuerzburg during the
Summer 2026 semester.

The work was completed as group coursework. The repository collects the code,
results, screenshots, maps, and documentation for the exercises that involved
software implementation, robotics tooling, and ROS2-based experimentation.

## Project Overview

| Folder | Topic | Summary |
| --- | --- | --- |
| `camera_calibration/` | OpenCV camera calibration | Webcam calibration with chessboard detection, intrinsic/extrinsic parameters, undistortion, live pose estimation, and ROS2/RViz2 pose visualization. |
| `odometry_laser_mapping/` | Odometry and laser simulation | Odometry comparison evidence and a ROS2 laser scan simulator that ray-casts through an occupancy grid map using Bresenham's line algorithm. |
| `laser_scan_mapping/` | Laser scan occupancy mapping | A custom ROS2 mapper that combines laser scans and known odometry poses to build and publish an occupancy grid map. |
| `icp_scan_matching/` | ICP scan matching | A custom 2D point-to-point ICP implementation for aligning laser scans and publishing an accumulated meta scan. |

Each folder is self-contained and includes its own README with build commands,
runtime instructions, implementation notes, and the relevant visual results.

## Context

The exercises cover core topics in mobile robotics:

- camera calibration and pose estimation,
- odometry correction and visualization,
- laser scan simulation,
- occupancy grid mapping,
- SLAM-related map generation,
- scan matching with ICP.

The goal of this repository is not to present a single production application,
but to preserve the implemented parts of the coursework in a clean and
reviewable form. Original assignment material was used only to contextualize
the READMEs; the folders focus on runnable code and submitted results.

## Technical Stack

The projects use:

- ROS2,
- C++ and Python,
- OpenCV,
- RViz2,
- Nav2 map tools,
- NumPy and SciPy for the ICP implementation.

Local verification was performed with ROS2 Jazzy. If another ROS2 distribution
is used, source the corresponding setup file before building.

## Building

From this repository root, all ROS2 packages can be built together:

```bash
source /opt/ros/jazzy/setup.bash
colcon build \
  --base-paths camera_calibration odometry_laser_mapping laser_scan_mapping icp_scan_matching
```

Then source the generated workspace setup:

```bash
source install/setup.bash
```

For details about individual executables, launch files, parameters, and
expected input topics, see the README inside each project folder.

## Notes

These projects were created for academic submission and learning. Some runtime
commands require recorded ROS2 bags, camera hardware, laser scan topics, or map
files that were part of the course environment.

