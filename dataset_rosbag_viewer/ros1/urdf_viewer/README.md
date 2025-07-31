# urdf_viewer

A ROS 1 package for visualizing URDF/XACRO robot models in RViz, with support for custom RViz configurations and image transport plugins. Tested with ROS1 noetic

## Features
- Standalone launch file for URDF/XACRO visualization
- Joint state publisher
- Robot state publisher with xacro support
- RViz integration with custom config
- Ready for image, laser scan and point cloud visualization

## Usage
```
Thefore yo need to install ROS 1 and execute the following commands:  
Start the roscore: `roscore`  
Start the rosbag: `rosbag play -l rosbags/night.bag`  
Launch the visualization: `roslaunch urdf_viewer/view_AMR_sensors.launch` 
```