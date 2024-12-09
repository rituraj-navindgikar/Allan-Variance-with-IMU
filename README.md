# Feature-Based Panoramic Image Stitching with Calibrated Cameras

## Overview

This project implements a pipeline for stitching panoramic images using feature-based matching and calibrated cameras. It leverages OpenCV and NumPy for image processing and ensures accurate perspective transformation to align multiple images into a seamless panorama

Additionally, the noise analysis of IMU data from the VectorNav 100 is incorporated to understand potential sources of error and validate sensor accuracy during the process

---

## Features

- **Feature Matching**: Matches key features between image pairs using BFMatcher and a ratio test for filtering
- **Perspective Transformation**: Aligns images using homographies and supports padded warping to maintain image integrity
- **Blending**: Combines overlapping image regions with masking and alpha blending for a smooth transition
- **Visualization**: Provides visual debugging for image matching and stitching processes
- **IMU Noise Analysis**: Includes insights into yaw, pitch, and roll data distributions, emphasizing Gaussian and skew-normal trends, with histograms for each parameter

---

## Repository Structure

- **main.ipynb**: Contains the primary workflow for loading images, detecting features, matching them, and stitching the panorama
- **utils.py**: Includes utility functions for tasks like loading images, matching features, perspective warping, and blending
- **data/**: Contains subfolders with test datasets:
  - **`cinder_wall/`**: Images with approximately 50% overlap
  - **`graphic_overlap_15/`**: Images with approximately 15% overlap
  - **`graphic_overlap_50/`**: Images with approximately 50% overlap

---

## Data Analysis Highlights

- **IMU Data Observations**:
  - Yaw and pitch exhibit Gaussian distributions, with minimal noise and no heavy-tailed behavior
  - Roll data shows a slight skew, indicative of a skew-normal distribution
  - Acceleration in the X-axis suggests a bimodal distribution due to varying conditions, while Y and Z follow Gaussian trends
  - Magnetic field data highlights minimal deviations and consistent noise patterns without significant drifts

- **Sources of Noise**:
  - Vibrations (e.g., machinery, traffic, external activities)
  - Sensor bias, including bias instability and angle random walk
  - Environmental factors, such as temperature variations and nearby magnetic/electronic interference

---

## How to Run

1. Clone the repository and ensure the required dependencies are installed
2. Use the datasets in the `data/` folder or your own images for testing
3. Open `main.ipynb` and follow the workflow to process your images and create the panorama
4. Use `utils.py` for debugging or extending the stitching process

---

## Future Work

- Improve feature detection and matching for challenging scenarios like low-texture areas
- Integrate real-time stitching support for video feeds
- Optimize the blending process for dynamic scenes

---

## Acknowledgments

- This project uses OpenCV for image processing and feature matching
- IMU noise analysis and insights inspired by Allan Variance studies and sensor data interpretation

---

Developed by  
**Rituraj Navindgikar**  
