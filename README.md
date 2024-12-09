# **IMU Noise Characterization with Allan Variance**

This project involves characterizing the noise of the **VNYMR VectorNav 100 IMU** using Allan Variance. It includes ROS 2 packages for IMU data acquisition and processing, along with MATLAB scripts for analysis. The repository is organized into three main sections: source code, data, and analysis.

---

## **Repository Structure**

### **1. `src/`**
- Contains ROS 2 packages:
  - **`imu_driver`**: ROS 2 driver for interfacing with the VectorNav VNYMR 100 IMU.
  - **`imu_msgs`**: ROS 2 message definitions used by the driver.

### **2. `data/`**
- Includes datasets of IMU recordings saved in ROS 2 bag format:
  - A 15-minute dataset for quick analysis.
  - A **5-hour dataset** for detailed outcomes ([Download here](https://drive.google.com/drive/folders/1-lNLGQ0DAIOL3fmlQ1-2_LytfiMF2h1J?usp=sharing)).

### **3. `analysis/`**
- Contains all MATLAB code for data analysis:
  - **Allan Variance computation** to characterize noise.
  - **Graphs and histograms** of IMU noise characteristics.
  - **ROS 2 bag-to-MATLAB conversion script**.

---

## **Getting Started**

### **1. Prerequisites**
Ensure the following are installed on your system:
- **ROS 2** (Humble or later)
- **MATLAB** (with required toolboxes for data processing)
- **Python 3** (for ROS 2 integration)

### **2. Build the ROS 2 Packages**
Build the `imu_driver` and `imu_msgs` packages:
```bash
colcon build --packages-select imu_msgs imu_driver --symlink-install
source install/setup.bash
```
### **3. Launch the IMU Driver**
```bash
ros2 launch imu_driver imu_launch.py
```
---

## Analysing IMU Data
### **1. Collect Data**
```bash
ros2 bag record -o imu_data /imu
```
### **2. Convert ROS bag to Matlab Format**

###  **Step 3: Run Allan Variance Analysis**
Load the converted data into MATLAB and run the Allan Deviation script. This will generate:

- Allan Variance graphs
- Histograms and statistical analysis of noise characteristics

run the `plotter.py` file for time-series plots and histograms of each IMU parameter, including:
- Yaw, pitch, roll
- Linear acceleration (acc_x, acc_y, acc_z)
- Angular velocity (gyro_x, gyro_y, gyro_z)
- Magnetic field (mag_x, mag_y, mag_z)
	

Developed By
Rituraj Navindgikar
