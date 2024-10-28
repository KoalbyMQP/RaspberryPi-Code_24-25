import numpy as np
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

try:
    import board
except NotImplementedError:
    print("Failed to import board when not running on Raspberry Pi")

class PressureSensor() :
    def __init__(self, isReal, sim=None, sensor_name="ForceTR"):
        self.isReal = isReal
        self.isConnected = True
        self.sim = sim
        self.sensor_name = sensor_name

        
