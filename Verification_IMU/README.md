# IMU and Encoder Verification Guide

This calibration process aligns IMU orientation data with encoder measurements by adjusting the angle offset. Follow these steps to ensure accurate synchronization:

1. **Run Verification Tools**: Execute `imu_orient.py` and `enc_final2.py` simultaneously to adjust the angle offset between the IMU and encoder.

2. **Data Collection**: Perform calibration movements to gather synchronized datasets from both devices.

3. **Analyze Calibration**: Run `plot_dataset_err.py` to calculate the Root Mean Squared Error (RMSE) and visualize the alignment errors between the IMU and encoder.

**Outputs**:
- **RMSE**: Provides a quantitative measure of calibration accuracy.
- **Plots**: Display the IMU and encoder angles over time along with the absolute error for visual assessment.

**Prerequisites**:
- Ensure all required Python libraries (`pandas`, `matplotlib`, `numpy`) are installed before starting the calibration process.

---

# Usage Example

```bash
python imu_orient.py &
python enc_final2.py &
# After collecting data
python plot_dataset_err.py
