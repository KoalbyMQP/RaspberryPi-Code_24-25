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

        if isReal:
            try:
                i2c = board.I2C()  # uses board.SCL and board.SDA
                self.sensor = mcc128(i2c)
            except:
                print("No IMU detected, disabling IMU")
                self.isConnected = False

    def zero(self):
            if self.isReal:
                self.zeroAngles = self.getDataRaw()
            else:
                raise NotImplementedError("IMU zero not implemented for simulation IMU")
            
    def getData(self):
        if not self.isConnected:
            return [0, 0, 0, 0, 0, 0]
        if self.isReal:
            self.data = [
                self.sensor.force[0],
                -self.sensor.force[2],
                self.sensor.force[1]
            ]
        elif self.sim:
            # Different signal names for each IMU based on imu_name
            prefix = f"Force{self.sensor_name}"
            self.data = [
                self.sim.getFloatSignal(f"{prefix}_forceX"),
                self.sim.getFloatSignal(f"{prefix}_forceY"),
                self.sim.getFloatSignal(f"{prefix}_forceZ"),
            ]
            # Handle cases where signals may not be available (default to 0 if no data)
            self.data = [0 if dataPoint is None else dataPoint for dataPoint in self.data]
        else:
            self.data = [0, 0, 0, 0, 0, 0]  # Fallback if sim is not set in simulation mode
        return self.data
    

class ForceManager():
    def __init__(self, isReal, sim=None):
        """
        Initialize the IMU manager for handling multiple IMUs.
        
        Parameters:
        - isReal: Boolean indicating if the IMUs are real or simulated.
        - sim: The simulation client for accessing simulation signals.
        """
        self.isReal = isReal
        self.sim = sim
        self.sensor_names = ["RTR", "RTL", "RBR", "RBL", "LTR", "LTL", "LBR", "LBL"]
        self.sensors = {name: PressureSensor(isReal, sim, name) for name in self.sensor_names}

    def getAllForces(self):
        """
        Retrieve data for all IMUs.
        
        Returns:
        - A dictionary where keys are IMU names and values are the IMU data.
        """
        data = {name: self.sensors[name].getData() for name in self.sensor_names}
        return data
    
    def pressurePerSensor(self):
        pressures = [0, 0, 0, 0, 0, 0, 0, 0]
        data = self.getAllForces()
        pressures[0] = math.sqrt( (data["RTR"][0] ** 2) + (data["RTR"][1] ** 2) + (data["RTR"][2] ** 2) )
        pressures[1] = math.sqrt( (data["RTL"][0] ** 2) + (data["RTL"][1] ** 2) + (data["RTL"][2] ** 2) )
        pressures[2] = math.sqrt( (data["RBR"][0] ** 2) + (data["RBR"][1] ** 2) + (data["RBR"][2] ** 2) )
        pressures[3] = math.sqrt( (data["RBL"][0] ** 2) + (data["RBL"][1] ** 2) + (data["RBL"][2] ** 2) )
        pressures[4] = math.sqrt( (data["LTR"][0] ** 2) + (data["LTR"][1] ** 2) + (data["LTR"][2] ** 2) )
        pressures[5] = math.sqrt( (data["LTL"][0] ** 2) + (data["LTL"][1] ** 2) + (data["LTL"][2] ** 2) )
        pressures[6] = math.sqrt( (data["LBR"][0] ** 2) + (data["LBR"][1] ** 2) + (data["LBR"][2] ** 2) )
        pressures[7] = math.sqrt( (data["LBL"][0] ** 2) + (data["LBL"][1] ** 2) + (data["LBL"][2] ** 2) )
        return pressures