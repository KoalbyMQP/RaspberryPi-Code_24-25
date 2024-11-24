# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_bno055
import adafruit_tca9548a
import matplotlib.pyplot as plt

# Create I2C bus and TCA9548A object
i2c = board.I2C()
tca = adafruit_tca9548a.TCA9548A(i2c)

# Initialize the IMU sensors
imu1 = adafruit_bno055.BNO055_I2C(tca[6])
imu2 = adafruit_bno055.BNO055_I2C(tca[7])

# Data storage
time_data = []
imu1_data = {"yaw": [], "roll": [], "pitch": []}
imu2_data = {"yaw": [], "roll": [], "pitch": []}

# Start time for timestamps
start_time = time.time()

# Collect data for 100 iterations
for i in range(100):
    current_time = time.time() - start_time
    
    # Get IMU1 data
    euler1 = imu1.euler
    if euler1:  # Ensure data is valid
        if(euler1[0]>180):
            imu1_data["yaw"].append(euler1[0] - 360)
        else:
            imu1_data["yaw"].append(euler1[0])
        imu1_data["roll"].append(euler1[1])
        imu1_data["pitch"].append(euler1[2])

    # Get IMU2 data
    euler2 = imu2.euler
    if euler2:
        if(euler2[0]>180):
            imu2_data["yaw"].append(euler2[0] - 360)
        else:
            imu2_data["yaw"].append(euler2[0])
        imu2_data["roll"].append(euler2[1])
        imu2_data["pitch"].append(euler2[2])
        
    # Record time
    time_data.append(current_time)
    
    # Print to console (optional)
    print(f"Time: {current_time:.2f}s | IMU1 - Yaw: {euler1[0]:.2f}, Roll: {euler1[1]:.2f}, Pitch: {euler1[2]:.2f} | IMU2 - Yaw: {euler2[0]:.2f}, Roll: {euler2[1]:.2f}, Pitch: {euler2[2]:.2f}")

    time.sleep(0.05)  # 50ms delay

# Plot function
def plot_data(time_data, data, imu_name):
    plt.figure(figsize=(8, 5))
    plt.plot(time_data, data["yaw"], label="Yaw", color='r')
    plt.plot(time_data, data["roll"], label="Roll", color='g')
    plt.plot(time_data, data["pitch"], label="Pitch", color='b')
    plt.xlabel("Time (s)")
    plt.ylabel("Angle (Degrees)")
    plt.title(f"{imu_name} Yaw, Roll, Pitch vs Time")
    plt.ylim(-180, 180)
    plt.legend()
    plt.grid()
    plt.savefig(f"{imu_name.lower()}_roll_data.png")  # Save plot to file
    print(f"Plot saved as '{imu_name.lower()}_data.png'")

# Plot and save data for each IMU
plot_data(time_data, imu1_data, "IMU1")
plot_data(time_data, imu2_data, "IMU2")