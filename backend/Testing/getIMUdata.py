# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT
# This example shows using two TSL2491 light sensors attached to TCA9548A channels 0 and 1.
# Use with other I2C sensors would be similar.
import time
import board
import adafruit_bno055
import adafruit_tca9548a
# Create I2C bus as normal
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
# Create the TCA9548A object and give it the I2C bus
tca = adafruit_tca9548a.TCA9548A(i2c)
# For each sensor, create it using the TCA9548A channel instead of the I2C object
imu1 = adafruit_bno055.BNO055_I2C(tca[6])
imu2 = adafruit_bno055.BNO055_I2C(tca[7])
# After initial setup, can just use sensors as normal.
while True:
    yaw1, roll1, pitch1 = imu1.euler
    yaw2, roll2, pitch2 = imu2.euler
    print("IMU 1 yaw: ", yaw1, " IMU 1 roll: ", roll1, " IMU 1 pitch: ", pitch1, "   IMU 2 yaw: ", yaw2, " IMU 2 roll: ", roll2, " IMU 2 pitch: ", pitch2)
    time.sleep(0.1)