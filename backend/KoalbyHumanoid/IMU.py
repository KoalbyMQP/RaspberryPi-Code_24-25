import adafruit_bno055
import numpy
import math

class IMU():
    def __init__(self, isReal, client_id=None):
        self.isReal = isReal
        self.client_id = client_id
        
        if isReal:
            import board
            i2c = board.I2C()  # uses board.SCL and board.SDA
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            
            self.zero = getData() # angles/accelerations that correspond to home position
        else:
            from backend.Simulation import sim as vrep

    def zero(self):
        if self.isReal:
            self.zero = getData()
        else:
            raise NotImplementedError("IMU zero not implemented for simulation IMU")

    # x axis is toward Ava's Left, y axis is up, z axis is toward Ava's front, all from the center of Ava
    # getData returns [x angle (rad), y angle (rad), z angle (rad), x acceleration (m/s^2), y acceleration (m/s^2), z acceleration (m/s^2)]
    def getData(self): 
        if self.isReal:
            # in terms of right hand rule convention for positive directions:
            # sensor.euler returns: (yaw (opposite convention), roll (normal convention), pitch (opposite convention))
            # yaw is in range 0 to 360, roll is -90 to 90 (rolling the IMU 180 degrees results in the same roll reading), pitch is -180 to 180
            # sensor.acceleration returns: (x acceleration (normal convention), z acceleration (opposite convention), y acceleration (normal convention))
            yaw, roll, pitch = self.sensor.euler
            
            self.data = np.subtract( # subtract reading from zero to get reading relative for 'zero position'
                [
                    math.radians(-pitch),
                    math.radians(-(yaw if yaw <= 180 else yaw - 360)), # mapping from 0 to 360 to -180 to 180
                    math.radians(roll),
                    self.sensor.acceleration[0],
                    -self.sensor.acceleration[2],
                    self.sensor.acceleration[1]
                ],
                self.zero
            )
        else:
            # gyro angles follow normal right hand rule conventions
            # angles go from -pi to pi rad
            self.data = [vrep.simxGetFloatSignal(self.client_id, "gyroX", vrep.simx_opmode_buffer)[1], # pitch
                    vrep.simxGetFloatSignal(self.client_id, "gyroY", vrep.simx_opmode_buffer)[1], # yaw
                    vrep.simxGetFloatSignal(self.client_id, "gyroZ", vrep.simx_opmode_buffer)[1], # roll
                    vrep.simxGetFloatSignal(self.client_id, "accelX", vrep.simx_opmode_buffer)[1],
                    vrep.simxGetFloatSignal(self.client_id, "accelY", vrep.simx_opmode_buffer)[1],
                    vrep.simxGetFloatSignal(self.client_id, "accelZ", vrep.simx_opmode_buffer)[1]]

        return self.data