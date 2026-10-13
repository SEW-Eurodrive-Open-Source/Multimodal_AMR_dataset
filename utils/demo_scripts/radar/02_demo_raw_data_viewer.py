"""
SPDX-FileCopyrightText: Copyright (c) 2025 SEW-EURODRIVE, Author: Leon Rafael Schönfeld
SPDX-License-Identifier: MIT
"""

"""
Demo: Raw Radar Data Processing and Visualization (Range-Azimuth and Range-Doppler)

This script demonstrates how to load, process, and visualize raw radar ADC data using the RadarSensor class.
It showcases the creation of range-azimuth and range-doppler images from .mat files containing unprocessed ADC data.

The availability of raw ADC data in this dataset enables advanced users to develop and test their own radar signal processing algorithms from scratch.
The provided RadarSensor class is intended as a starting point for your own research and development—feel free to extend or modify it to implement custom processing, detection, or visualization methods on the available radar raw data.
"""
from radar_sensor import RadarSensor

def main():
    mat_file_path = "your/path/to/matflie.mat"   # TODO: update path
    radar = RadarSensor()

    # Load and process raw ADC data
    c_cube = radar.create_c_cube(mat_file_path)
    print(f"Loaded c_cube with shape: {c_cube.shape}")

    # Apply 2D windowing
    c_cube = radar.apply_2d_window(c_cube)

    # Create range-azimuth and range-doppler images
    ra_image_db = radar.create_range_azimuth_image(c_cube)
    rd_image_db = radar.create_range_doppler_image(c_cube)

    # Crop to valid range bins
    ra_image_db = radar.crop_to_valid_range(ra_image_db)
    rd_image_db = radar.crop_to_valid_range(rd_image_db)

    # Plot Range-Azimuth
    radar.plot_image(ra_image_db, "Range-Azimuth Image (dB)", "Range Bin", "Azimuth Bin")

    # Plot Range-Doppler
    radar.plot_image(rd_image_db, "Range-Doppler Image (dB)", "Range Bin", "Doppler Bin")

if __name__ == "__main__":
    main()
