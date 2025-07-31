"""
SPDX-FileCopyrightText: Copyright (c) 2025 SEW-EURODRIVE, Author: Leon Rafael Schönfeld
SPDX-License-Identifier: MIT
"""

import os
import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import cv2
import yaml

class RadarSensor:
    """
    RadarSensor class for working with both raw radar ADC data and on-chip processed point cloud data.

    This class provides methods to:
      - Load and process raw ADC data from .mat files (e.g., create complex data cubes, apply windowing, FFTs)
      - Visualize range-azimuth and range-doppler images
      - Load, filter, and visualize radar point cloud data (as processed on-chip, e.g., by the Texas Instruments AWR1843AOP)
      - Transform point clouds between sensor frames and project them onto RGB images using camera calibration

    The RadarSensor class is intended as a starting point for developing your own radar data processing and visualization algorithms on the SEW Dataset 2025, 
    which provides both raw ADC and processed point cloud data.
    """

    def __init__(self):
        """
        Initialize RadarSensor with default radar parameters.
        """
        # Radar parameters
        self.NUM_TX = 3
        self.NUM_RX = 4
        self.NUM_CHIRP_LOOPS = 64
        self.NUM_CSAMPLES = 256
        self.AZIMUTH_INTERP_SIZE = 128
        self.VALID_RANGE_BINS = 198

    def load_adc_cube(self, mat_file_path):
        """
        Load ADC data cube from a .mat file.
        Args:
            mat_file_path (str): Path to the .mat file.
        Returns:
            np.ndarray: ADC data cube.
        """
        if not os.path.exists(mat_file_path):
            raise FileNotFoundError(f"MAT file not found: {mat_file_path}")
        mat_data = loadmat(mat_file_path)
        return mat_data["gt"]

    def create_c_cube(self, mat_file_path):
        """
        Create a complex data cube from ADC data.
        Args:
            mat_file_path (str): Path to the .mat file.
        Returns:
            np.ndarray: Complex data cube.
        """
        adc_data = self.load_adc_cube(mat_file_path)
        c_cube = adc_data.astype(np.complex64)
        return c_cube

    def apply_2d_window(self, c_cube):
        """
        Apply a 2D windowing function to the data cube.
        Args:
            c_cube (np.ndarray): Complex data cube.
        Returns:
            np.ndarray: Windowed data cube.
        """
        # Create a 2D windowing function
        doppler_window = np.hanning(self.NUM_CHIRP_LOOPS)
        range_window = np.hanning(self.NUM_CSAMPLES)
        window_2d = np.outer(doppler_window, range_window)
        # Apply windowing to each TX/RX channel
        for tx in range(self.NUM_TX):
            for rx in range(self.NUM_RX):
                c_cube[:, tx, rx, :] *= window_2d
        return c_cube
    
    def crop_to_valid_range(self, image):
        """
        Crop the image to valid range bins.
        Args:
            image (np.ndarray): Input image.
        Returns:
            np.ndarray: Cropped image.
        """
        return image[:, :self.VALID_RANGE_BINS]

    def rangebin2meter(self, range_bin, bin_width=0.047, offset=-0.015):
        """
        Convert range bin index to meters.
        Args:
            range_bin (int or np.ndarray): Range bin index.
            bin_width (float): Width of each bin in meters.
            offset (float): Offset in meters.
        Returns:
            np.ndarray: Range in meters.
        """
        range_m = bin_width * range_bin + offset
        return np.clip(range_m, a_min=1e-3, a_max=None)
    
    def idx2deg(self, angle_idx, interp_size=128):
        """
        Convert angle index to degrees using calibration parameters.
        Args:
            angle_idx (int or np.ndarray): Angle index.
            interp_size (int): Interpolation size.
        Returns:
            np.ndarray: Angle in degrees.
        """
        idx_norm = (angle_idx / (interp_size - 1)) * 2 - 1  # idx_norm ∈ [-1, 1]
        # Fit parameters (from calibration)
        A = 37.542     # Output scaling factor (degrees)
        B = 1.312      # Input scaling factor (dimensionless)
        # Apply non-linear mapping with clipping to avoid invalid arcsin input
        arg = np.clip(B * idx_norm, -1, 1)
        # Return angle in degrees
        return A * np.arcsin(arg)

    def create_range_azimuth_image(self, c_cube):
        """
        Create a range-azimuth image from the data cube.
        Args:
            c_cube (np.ndarray): Complex data cube.
        Returns:
            np.ndarray: Range-azimuth image in dB.
        """
        ra_fft = np.fft.fftshift(
            np.fft.fftn(
                c_cube,
                s=(self.AZIMUTH_INTERP_SIZE, self.NUM_CSAMPLES),
                axes=(1, 3),
                norm="ortho"
            ),
            axes=1
        )
        # Take mean over chirps and receivers for visualization
        ra_image = np.abs(ra_fft).mean(axis=(0, 2))
        ra_image_db = 10 * np.log10(ra_image * 6 + 1e-6)
        return ra_image_db

    def create_range_doppler_image(self, c_cube):
        """
        Create a range-Doppler image from the data cube.
        Args:
            c_cube (np.ndarray): Complex data cube.
        Returns:
            np.ndarray: Range-Doppler image in dB.
        """
        rd_fft = np.fft.fftshift(
            np.fft.fftn(
                c_cube,
                s=(self.NUM_CHIRP_LOOPS, self.NUM_CSAMPLES),
                axes=(0, 3),
                norm="ortho"
            ),
            axes=0
        )
        # Take mean over TX and RX for visualization
        rd_image = np.abs(rd_fft).mean(axis=(1, 2))
        rd_image_db = 10 * np.log10(rd_image + 1e-6)
        return rd_image_db

    def plot_image(self, image, title, xlabel, ylabel):
        """
        Plot a 2D image with colorbar.
        Args:
            image (np.ndarray): Image to plot.
            title (str): Plot title.
            xlabel (str): X-axis label.
            ylabel (str): Y-axis label.
        """
        plt.figure()
        plt.imshow(image, aspect='auto', cmap='jet', origin='lower')
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.colorbar(label="dB")
        plt.show()

    # --- Point cloud data ---
    def load_pointcloud(self, csv_path):
        """
        Load radar point cloud from a CSV with columns:
        x, y, z, intensity, doppler

        Returns: np.ndarray of shape (N, 5)
        """
        if not os.path.exists(csv_path):
            raise FileNotFoundError(f"CSV file not found: {csv_path}")
        
        pc = np.loadtxt(csv_path, delimiter=',')  # Assumes no header
        if pc.shape[1] != 5:
            raise ValueError(f"Expected 5 columns (x, y, z, intensity, doppler), got {pc.shape[1]}")
        
        return pc

    def filter_pointcloud(self, pc, field_ranges):
        """
        Filter point cloud by min/max values for each field.
        pc: np.ndarray of shape (N, 5)
        field_ranges: dict with keys in ['x', 'y', 'z', 'intensity', 'doppler']
                    and values as (min, max) tuples
        Returns: filtered point cloud (N_filtered, 5)
        """
        field_indices = {
            'x': 0,
            'y': 1,
            'z': 2,
            'intensity': 3,
            'doppler': 4
        }

        mask = np.ones(pc.shape[0], dtype=bool)
        for field, (min_val, max_val) in field_ranges.items():
            idx = field_indices[field]
            mask &= (pc[:, idx] >= min_val) & (pc[:, idx] <= max_val)
        
        return pc[mask]
    
    def plot_pointcloud_3d(self, pc, color_by='intensity', cmap='viridis'):
        """
        Plot radar point cloud in 3D.
        Color by 'intensity' or 'doppler'.

        Parameters:
        - pc: np.ndarray of shape (N, 5)
        - color_by: 'intensity' or 'doppler'
        - cmap: matplotlib colormap
        """
        if color_by not in ['intensity', 'doppler']:
            raise ValueError("color_by must be 'intensity' or 'doppler'")

        x = pc[:, 0]
        y = pc[:, 1]
        z = pc[:, 2]
        c = pc[:, 3] if color_by == 'intensity' else pc[:, 4]

        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(x, y, z, c=c, cmap=cmap, s=10, alpha=0.8)

        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")
        ax.set_title(f"3D Radar Point Cloud colored by {color_by}")

        cb = plt.colorbar(sc, ax=ax, pad=0.1, shrink=0.8)
        cb.set_label(color_by.capitalize())

        plt.show()

    def transform_pointcloud(self, pc, transform_matrix):
        """
        Transform point cloud to another frame using a 4x4 transformation matrix.
        pc: np.ndarray of shape (N, 5) (x, y, z, intensity, doppler)
        transform_matrix: np.ndarray of shape (4, 4)
        Returns: transformed point cloud (N, 5)
        """
        # Convert to homogeneous coordinates
        points_hom = np.ones((pc.shape[0], 4))
        points_hom[:, :3] = pc[:, :3]
        # Apply transformation
        points_transformed = (transform_matrix @ points_hom.T).T
        # Replace xyz in pc with transformed xyz
        pc_transformed = pc.copy()
        pc_transformed[:, :3] = points_transformed[:, :3]
        return pc_transformed
    
    def load_camera_intrinsics_from_yaml(self, yaml_path):
        """
        Load camera intrinsics from a ROS ost.yaml file.
        Returns: K (3x3 np.array), D (1x5 np.array), image_width, image_height
        """
        with open(yaml_path, 'r') as f:
            calib = yaml.safe_load(f)
        K = np.array(calib['camera_matrix']['data']).reshape(3, 3)
        D = np.array(calib['distortion_coefficients']['data'])
        width = calib.get('image_width', None)
        height = calib.get('image_height', None)
        return K, D, width, height
    
    def project_points_to_image(self, points_3d, K):
        """
        Project 3D points (N, 3) in camera frame to 2D image coordinates using camera intrinsics K.
        K: 3x3 camera intrinsic matrix
        Returns: (N, 2) array of pixel coordinates
        """
        # Only use points in front of the camera (z > 0)
        mask = points_3d[:, 2] > 0
        points_3d = points_3d[mask]
        # Project
        points_2d = (K @ points_3d.T).T
        points_2d = points_2d[:, :2] / points_2d[:, 2:3]
        return points_2d, mask
    