import time
import math
from backend.KoalbyHumanoid.IMU import IMU

print("Initializing...")

imu = IMU(True)

while True:
    print(imu.getData())
    
    # To have print update the same line in the console:
    # print("{}                                \r".format(imu.getData()), end="")

    time.sleep(0.01)