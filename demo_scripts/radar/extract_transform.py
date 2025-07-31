"""
SPDX-FileCopyrightText: Copyright (c) 2025 SEW-EURODRIVE, Author: Leon Rafael Schönfeld
SPDX-License-Identifier: MIT
"""

"""
Utility: Extract 4x4 Transformation Matrix from URDF/Xacro

Use this utility script to extract a 4x4 transformation matrix from the sensor definitions in
../dataset_rosbag_viewer/ros1/urdf_viewer/urdf/AMR_sensors.xacro in order to transform data from one modality to another (e.g., radar, RGB camera, etc.).

The script provides an example of how to chain transformations, such as computing the transform from radar_link to rgb_link by combining the appropriate matrices.
"""
import numpy as np

def rpy_to_matrix(roll, pitch, yaw):
    """Convert RPY to a 3×3 rotation matrix."""
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx  # Intrinsic XYZ rotation

def make_transform(xyz, rpy):
    """Construct 4×4 transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T

def print_matrix(label, matrix):
    print(f"{label}:\n{np.round(matrix, 4)}\n")

if __name__ == "__main__":
    # base_link → radar_link
    T_base_to_radar = make_transform([0.187, 0.204, 0.569], [3.14159, 0, 0])
    
    # base_link → rgb_link
    T_base_to_rgb = make_transform([0.198, -0.01, 0.595], [0, 0, 0])
    
    # radar_link → base_link
    T_radar_to_base = np.linalg.inv(T_base_to_radar)
    
    # radar_link → rgb_link = (radar → base) × (base → rgb)
    T_radar_to_rgb = T_radar_to_base @ T_base_to_rgb

    # Output
    print_matrix("Transformation matrix: radar_link → rgb_link", T_radar_to_rgb)