"""The Motor class hold all information for an abstract motor on the physical robot. It is used to interface with the
arduino which directly controls the motors"""
import time
from backend.Simulation import sim as vrep
from backend.KoalbyHumanoid.PID import PID

class Motor():
    def __init__(self, is_real, motor_id, name, twist, M, angle_limit=None, serial=None, pidGains=None, client_id=None, handle=None):
        self.is_real = is_real
        self.motor_id = motor_id
        self.name = name
        self.twist = twist
        self.M = M
        self.prevTime = time.perf_counter()
        self.target = (0, 'P')

        if is_real:
            self.angle_limit = angle_limit
            self.serial = serial
        else:
            self.pidGains = pidGains
            self.client_id = client_id
            self.handle = handle
            self.simMovePID = PID(self.pidGains[0], self.pidGains[1], self.pidGains[2])

        self.target = (0, 'P')
        self.theta = None

    def get_position(self):
        if self.is_real:
            self.arduino_serial.send_command_arr([5, self.motor_id])
            current_position = self.arduino_serial.read_command()
        else:
            current_position = vrep.simxGetJointPosition(self.client_id, self.handle, vrep.simx_opmode_buffer)[1]
        return current_position
    
    def set_position(self, position):
        if self.is_real:
            self.arduino_serial.send_command_arr([10, self.motor_id, position])
        else:
            """sends a desired motor position to the Simulation"""
            self.theta = self.get_position()
            error = position - self.theta
            self.simMovePID.setError(error)
            self.effort = self.simMovePID.calculate()
            self.set_velocity(self.effort)
            
    def set_position_time(self, position, time):
        if self.is_real:
            """sends a desired motor position to the arduino <to be executed in a set amount of time>"""
            self.arduino_serial.send_command_arr([11, self.motor_id, position, time])
        else:
            raise NotImplementedError("set_position_time is not supported in simulation")

    def set_torque(self, on):
        if self.is_real:
            """sets the torque of the motor on or off based on the 'on' param"""
            self.arduino_serial.send_command_arr([20, self.motor_id, int(on)])
        else:
            raise NotImplementedError("set_torque in simulation motor not implemented")

    def get_velocity(self):
        raise NotImplementedError("get_velocity in Motor not implemented")

    def set_velocity(self, velocity):
        if self.is_real:
            self.arduino_serial.send_command_arr([40, self.motor_id, velocity])
        else:
            vrep.simxSetJointTargetVelocity(self.client_id, self.handle, velocity, vrep.simx_opmode_streaming)

    def move(self, target="TARGET"):
        if time.perf_counter() - self.prevTime > 0.01:
            if target == "TARGET":
                target = self.target
            if target[1] == 'P':
                self.set_position(target[0])
            elif target[1] == 'V':
                self.set_velocity(target[0])
            else:
                raise Exception("Invalid goal")
        self.prevTime = time.perf_counter()