# IMU and Encoder Verification Guide

This project verifies the alignment between IMU orientation data and encoder measurements using the **Angle Axis Screw Method**. By running `imu_orient.py` and `enc_final2.py` concurrently, synchronized datasets are collected during **10 full rotations (360 degrees)**. After verification, execute `plot_dataset_err.py` to compute the **Root Mean Squared Error (RMSE)** and visualize the discrepancies between the IMU and encoder.

![Absolute Error Plot](plots/absolute_error_plot.png)

## Verification Process

1. **Data Collection**:
   - **Run Verification Scripts**: Execute `imu_orient.py` and `enc_final2.py` simultaneously to adjust the angle offset and collect synchronized IMU and encoder data.
     ```bash
     python imu_orient.py &
     python enc_final2.py &
     ```
   - **Perform Rotations**: Conduct **10 full rotations (3600 degrees total)** to gather comprehensive data.

2. **Error Analysis**:
   - **Run Analysis Script**: After data collection, execute `plot_dataset_err.py` to calculate the **RMSE** and generate visualization plots.
     ```bash
     python plot_dataset_err.py
     ```
   - **Review Results**: The RMSE is approximately **6 degrees**, indicating the average deviation between the IMU and encoder measurements over the rotations.

## Angle Axis Screw Method

The **Angle Axis Screw Method** mathematically aligns rotational data by determining the optimal rotation (angle and axis) that minimizes the error between two datasets. This method involves:

- **Defining Rotations**: Representing orientations from the IMU and encoder as rotation matrices.
- **Calculating Screw Parameters**: Solving for the screw axis (a combination of rotation axis and angle) that best fits the data.
- **Minimizing Error**: Utilizing optimization techniques to minimize the discrepancy, quantified by RMSE.

**Mathematical Representation**:
\[
\text{RMSE} = \sqrt{\frac{1}{n} \sum_{i=1}^{n} (\theta_{\text{Encoder},i} - \theta_{\text{IMU},i})^2}
\]
Where:
- \(\theta_{\text{Encoder},i}\) is the angle measured by the encoder at the \(i^{th}\) timestamp.
- \(\theta_{\text{IMU},i}\) is the angle measured by the IMU at the \(i^{th}\) timestamp.
- \(n\) is the total number of measurements.

## Uploading the Plot Image

To include your latest dataset and plot image in your repository:

1. **Place the Plot Image**:
   - Save your plot image (`absolute_error_plot.png`) in the `plots` directory.

2. **Commit and Push to Repository**:
   ```bash
   git add plots/absolute_error_plot.png
   git commit -m "Add latest absolute error plot"
   git push
