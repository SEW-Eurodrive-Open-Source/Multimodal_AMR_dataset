"""
SPDX-FileCopyrightText: Copyright (c) 2025 SEW-EURODRIVE, Author: Leon Rafael Schönfeld
SPDX-License-Identifier: MIT
"""

"""
Demo: Radar Point Cloud to RGB Image Projection

This script demonstrates how to project a radar point cloud onto an undistorted RGB image using the RadarSensor class.
It performs the following steps:
 1. Loads and undistorts the RGB image using camera calibration (ost.yaml)
 2. Loads and filters the radar point cloud
 3. Transforms the point cloud from radar_link to rgb_link using a 4x4 matrix
 4. Converts the point cloud from ROS to OpenCV camera frame
 5. Projects the 3D points onto the image using camera intrinsics
 6. Plots the RGB image with projected radar points colored by distance
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt
from radar_sensor import RadarSensor

def main():
    # --- 1. Load and undistort RGB image ---
    rgb_image_path = "your/path/toimge.jpg"  # TODO: update path
    ost_yaml_path = "your/path/to/calibration_files/RGB_intrinsics/ost.yaml" # TODO: update path
    radar = RadarSensor()
    rgb_img = cv2.imread(rgb_image_path, cv2.IMREAD_COLOR)
    if rgb_img is None:
        raise FileNotFoundError(f"Could not load image: {rgb_image_path}")
    K, D, width, height = radar.load_camera_intrinsics_from_yaml(ost_yaml_path)
    rgb_img_undist = cv2.undistort(rgb_img, K, D)
    rgb_img_undist = cv2.cvtColor(rgb_img_undist, cv2.COLOR_BGR2RGB)

    # --- 2. Load and filter radar point cloud ---
    radar_csv_path = "your/path/to/point_cloud.csv"  # TODO: update path
    pc = radar.load_pointcloud(radar_csv_path)
    ranges = {
        'x': (0, 10),
        'y': (-5, 5),
        'z': (-5, 5),
        'intensity': (0.0, 50.0),
        'doppler': (-5, 5)
    }
    pc_filtered = radar.filter_pointcloud(pc, ranges)

    # --- 3. Transform point cloud from radar_link to rgb_link, transform matrix extracted with extract_transform.py ---
    radar_to_rgb = np.array([
        [1.0,  0.0,  0.0,   0.011],
        [0.0, -1.0,  0.0,   0.214],
        [0.0, -0.0, -1.0,  -0.026],
        [0.0,  0.0,  0.0,   1.0]
    ])
    pc_rgb = radar.transform_pointcloud(pc_filtered, radar_to_rgb)

    # --- 4. Convert from ROS camera frame to OpenCV camera frame ---
    points_ros = pc_rgb[:, :3]
    points_cv = np.zeros_like(points_ros)
    points_cv[:, 0] = -points_ros[:, 1]      # x_cv = -y_ros
    points_cv[:, 1] = -points_ros[:, 2]      # y_cv = -z_ros
    points_cv[:, 2] = points_ros[:, 0]       # z_cv = x_ros

    # --- 5. Project 3D points to image ---
    points_2d, mask = radar.project_points_to_image(points_cv, K)

    # Remove points outside the image
    h, w = rgb_img_undist.shape[:2]
    in_img = (
        (points_2d[:, 0] >= 0) & (points_2d[:, 0] < w) &
        (points_2d[:, 1] >= 0) & (points_2d[:, 1] < h)
    )
    points_2d_in = points_2d[in_img]

    # --- 6. Plot the RGB image with projected radar points colored by distance ---
    distances = points_cv[mask][in_img, 2]
    plt.figure(figsize=(10, 8))
    im = plt.imshow(rgb_img_undist)
    sc = plt.scatter(points_2d_in[:, 0], points_2d_in[:, 1], s=50, c=distances, cmap='plasma', alpha=0.7, vmin=0, vmax=5, label='Radar points')
    plt.title("Radar Points Projected onto Undistorted RGB Image (colored by distance)")
    plt.xlabel("u [px]")
    plt.ylabel("v [px]")
    plt.colorbar(sc, label='Distance [m]', shrink=0.8, pad=0.02)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
