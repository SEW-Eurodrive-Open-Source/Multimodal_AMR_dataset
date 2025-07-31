# urdf_viewer

A ROS 2 package for visualizing URDF/XACRO robot models in RViz2, with support for custom RViz configurations and image transport plugins. Tested with ROS2 humble

## Features
- Standalone launch file for URDF/XACRO visualization
- Joint state publisher
- Robot state publisher with xacro support
- RViz2 integration with custom config
- Ready for image, laser scan and point cloud visualization

## Dependencies
Make sure you have the following dependencies installed in sourced ROS2 environment:

```
sudo apt install ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-image-transport-plugins ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-compressed-image-transport ros-$ROS_DISTRO-joint-state-publisher
```

## Build
Build the package using colcon:

```
colcon build --packages-select urdf_viewer
```

## Usage
Source your workspace and launch the viewer:

```
source install/setup.bash
ros2 launch urdf_viewer urdf_viewer.launch.py
ros2 bag play <rosbag>
```
