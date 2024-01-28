from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class IMU():
    def __init__(self, isReal, sim=None):
        self.isReal = isReal
        self.sim = sim

    def getData(self):
        if self.isReal:
            # Ana pls help
            #self.data = ???
            pass
        else:
            self.data = [self.sim.getFloatSignal("gyroX"),
                    self.sim.getFloatSignal("gyroY"),
                    self.sim.getFloatSignal("gyroZ"),
                    self.sim.getFloatSignal("accelX"),
                    self.sim.getFloatSignal("accelY"),
                    self.sim.getFloatSignal("accelZ")]
        return self.data