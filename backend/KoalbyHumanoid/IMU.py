class IMU():
    def __init__(self, isReal, client_id=None):
        self.isReal = isReal
        self.client_id = client_id
        if not isReal:
            from backend.Simulation import sim as vrep

    def getData(self):
        if self.isReal:
            # Ana pls help
            #self.data = ???
            pass
        else:
            self.data = [vrep.simxGetFloatSignal(self.client_id, "gyroX", vrep.simx_opmode_buffer)[1],
                    vrep.simxGetFloatSignal(self.client_id, "gyroY", vrep.simx_opmode_buffer)[1],
                    vrep.simxGetFloatSignal(self.client_id, "gyroZ", vrep.simx_opmode_buffer)[1],
                    vrep.simxGetFloatSignal(self.client_id, "accelX", vrep.simx_opmode_buffer)[1],
                    vrep.simxGetFloatSignal(self.client_id, "accelY", vrep.simx_opmode_buffer)[1],
                    vrep.simxGetFloatSignal(self.client_id, "accelZ", vrep.simx_opmode_buffer)[1]]

        return self.data
