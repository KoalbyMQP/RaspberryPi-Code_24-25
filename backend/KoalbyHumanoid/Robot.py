import sys
import time
import math
import numpy as np

sys.path.append("D:/Documents/College/Humanoid MQP Project/RaspberryPi-Code_23-24")
sys.path.append("C:/Users/Gabriel/AppData/Local/Programs/Python/Python312/Lib/site-packages")
import backend.KoalbyHumanoid.Config as Config
import modern_robotics as mr
from backend.KoalbyHumanoid.Link import Link
from backend.KoalbyHumanoid.PID import PID
from backend.KoalbyHumanoid.ArduinoSerial import ArduinoSerial
from backend.KoalbyHumanoid.Motor import Motor
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from backend.KoalbyHumanoid import poe as poe
from backend.KoalbyHumanoid.Electromagnet import Electromagnet
from backend.KoalbyHumanoid.IMU import IMU, IMUManager
from backend.KoalbyHumanoid.PressureSensor import PressureSensor, ForceManager

TIME_BETWEEN_MOTOR_CHECKS = 2

class Robot():
    # Initialization methods

    def __init__(self, is_real):
        self.is_real = is_real
        if self.is_real:
            self.client = None
            self.sim = None
            self.client_id = None
            self.arduino_serial_init()
            self.motors = self.real_motors_init()
            
            self.imuPIDX = PID(0.5, 0.0, 0.2)
            self.imuPIDZ = PID(0.5, 0.0, 0.1)
            self.electromagnet = Electromagnet()
        else:
            self.checkCoppeliaSimResponding()

            self.client = RemoteAPIClient()
            self.sim = self.client.require('sim')
            self.motorMovePositionScriptHandle = self.sim.getScript(self.sim.scripttype_childscript, self.sim.getObject("./Chest_respondable"))
            self.motors = self.sim_motors_init()
            
            self.imuPIDX = PID(0.007, 0.0, 0.)
            self.imuPIDZ = PID(0.18, 0.0, 0.005)

        self.lastMotorCheck = time.time()
        self.imu_manager = IMUManager(self.is_real, sim=self.sim)
        self.ang_vel = [0, 0, 0]
        self.last_vel = [0, 0, 0]
        self.ang_accel = [0, 0, 0]
        self.balancePoint = np.array([0, 0, 0])
        self.rightFootBalancePoint = np.array([0, 0, 0])
        self.leftFootBalancePoint = np.array([0, 0, 0])
        self.primitives = []
        self.chain = self.chain_init()
        self.links = self.links_init()
        self.fused_imu = 0
        # Use IMUManager to manage multiple IMUs
        self.imu_manager = IMUManager(self.is_real, sim=self.sim)
        self.forceManager = ForceManager(self.is_real, sim=self.sim)
        self.feetCoP = [0, 0]
        self.CoPPIDX = PID(0.01, 0, 0)
        self.CoPPIDZ = PID(0.5, 0, 0)

        if not is_real:
            self.sim.startSimulation()
        print("Robot Created and Initialized")

    # Fuse IMU data from right_chest_imu, left_chest_imu and torso_imu
    def fuse_imu_data(self, right_chest_imu, left_chest_imu, torso_imu):
        """
        Fuses the IMU data from right chest, left chest, and torso.

        Args:
            right_chest_imu (array-like): [pitch, roll, yaw] from the right chest IMU.
            left_chest_imu (array-like): [pitch, roll, yaw] from the left chest IMU.
            torso_imu (array-like): [pitch, roll, yaw] from the torso IMU.

        Returns:
            np.array: Fused [pitch, roll, yaw] data for PID controller input.
        """
        self.fused_imu = np.mean([right_chest_imu, left_chest_imu, torso_imu], axis=0)
        return self.fused_imu
    
    def IMUBalance(self, Xtarget, Ztarget):
        imu_data = self.imu_manager.getAllIMUData()
        right_chest_imu = imu_data["RightChest"]
        left_chest_imu = imu_data["LeftChest"]
        torso_imu = imu_data["Torso"]

        # Fuse IMU data
        fused_data = self.fuse_imu_data(right_chest_imu, left_chest_imu, torso_imu)

        # Use the fused data for balance calculations
        xRot = fused_data[0]
        zRot = fused_data[2]  

        Xerror = Xtarget - xRot
        Zerror = Ztarget - zRot

        self.imuPIDX.setError(Xerror)
        self.imuPIDZ.setError(Zerror)

        newTargetX = self.imuPIDX.calculate()
        newTargetZ = self.imuPIDZ.calculate()

        # Apply corrections
        self.motors[13].target = (-newTargetZ, 'P')  # Adjust yaw
        self.motors[10].target = (newTargetX, 'P')  # Adjust pitch

        self.checkMotorsAtInterval(TIME_BETWEEN_MOTOR_CHECKS)

    def updateCoP(self): #get position of main pressure point on foot
        #foot dimensions are needed to calculate positions
        footWidth = 0.03 #5.36 # in cm - will change depending on actual locations on foot
        footLength = 0.11 #14.66 # in cm - will change depending on actual locations on foot

        #get pressure value from each pressure sensor
        data = self.forceManager.pressurePerSensor()

        rightTop= (data[0] + data[1]) / 2 #right foot
        rightBottom = (data[2] + data[3]) / 2 #right foot
        rightLeft = (data[1] + data[3]) / 2 #right foot
        rightRight = (data[0] + data[2]) / 2 #right foot
        rightCoPX = (rightRight*footWidth) / (rightRight + rightLeft)  #right foot CoP x location WRT right edge of foot
        rightCoPY = (rightTop*footLength) / (rightTop + rightBottom) #right foot CoP y location WRT top edge of foot

        leftTop= (data[4] + data[5]) / 2 #left foot
        leftBottom = (data[6] + data[7]) / 2 #left foot
        leftLeft = (data[5] + data[7]) / 2 #leftt foot
        leftRight = (data[4] + data[6]) / 2 #left foot
        leftCoPX = (leftRight*footWidth) / (leftLeft + leftRight) #left foot CoP x location WRT right edge of foot
        leftCoPY = (leftTop*footLength) / (leftTop + leftBottom) #left foot
        self.feetCoP[0] = (rightCoPX + leftCoPX) / 2 #average x for each foot
        self.feetCoP[1] = (rightCoPY + leftCoPY) / 2 #average y for each foot
        return self.feetCoP #values should be around half of footWidth and footLength

    def CoPBalance(self, CoPs):
        ErrorX = CoPs[0] - self.feetCoP[0]
        ErrorY = CoPs[1] - self.feetCoP[1]
        self.CoPPIDX.setError(ErrorX)
        self.CoPPIDZ.setError(ErrorY)
        targetX = self.CoPPIDX.calculate()
        targetZ = self.CoPPIDZ.calculate()

        self.motors[13].target = (targetX, 'V') #for hips side2side
        self.motors[10].target = (-targetZ, 'V') #for hips front2back

    def checkCoppeliaSimResponding(self):
        client = RemoteAPIClient()
        client._send({'func': '', 'args': ['']})
        if (client.socket.poll(1000) == 0):
            raise Exception("CoppeliaSim is not responding. Restart CoppeliaSim and try again.")
        else:
            client.__del__()

    def arduino_serial_init(self):
        self.arduino_serial = ArduinoSerial()
        self.initHomePos() # This initializes the robot with all the initial motor positions

    def chain_init(self):
        chain = {
            self.motors[24].name:self.motors[23],
            self.motors[23].name:self.motors[22],
            self.motors[22].name:self.motors[21],
            self.motors[21].name:self.motors[20],
            self.motors[20].name:self.motors[10],

            self.motors[19].name:self.motors[18],
            self.motors[18].name:self.motors[17],
            self.motors[17].name:self.motors[16],
            self.motors[16].name:self.motors[15],
            self.motors[15].name:self.motors[10],

            self.motors[10].name:self.motors[13],
            self.motors[13].name:self.motors[12],
            self.motors[12].name:self.motors[14],
            self.motors[14].name:self.motors[11],
            self.motors[11].name:"base",

            self.motors[4].name:self.motors[3],
            self.motors[3].name:self.motors[2],
            self.motors[2].name:self.motors[1],
            self.motors[1].name:self.motors[0],
            self.motors[0].name:"base",

            self.motors[9].name:self.motors[8],
            self.motors[8].name:self.motors[7],
            self.motors[7].name:self.motors[6],
            self.motors[6].name:self.motors[5],
            self.motors[5].name:"base"
        }
        return chain
    
    def links_init(self):
        links = list()
        for linksConfig in Config.links:
            link = Link(linksConfig[0], linksConfig[1])
            links.append(link)
        return links

    def sim_motors_init(self):
        motors = list()
        for motorConfig in Config.motors:
            print("Beginning to stream", motorConfig[3])
            handle = self.sim.getObject("/" + motorConfig[3])

            motor = Motor(False, motorConfig[0], motorConfig[3], motorConfig[6], motorConfig[7], pidGains=motorConfig[5], sim=self.sim, handle=handle)
            motor.theta = motor.get_position()
            motor.name = motorConfig[3]
            motors.append(motor)
        return motors
    
    def real_motors_init(self):
        motors = list()
        for motorConfig in Config.motors:
            motor = Motor(True, motorConfig[0], motorConfig[3], motorConfig[6], motorConfig[7], angle_limit=motorConfig[1], serial=self.arduino_serial)
            motors.append(motor)
        return motors
    
    # Motor Control Methods

    def getMotor(self, key):
        for motor in self.motors:
            if motor.motor_id == key:
                return motor

    def moveTo(self, motor, position):
        motor.move(position)
        
    def moveToTarget(self, motor):
        motor.move(motor.target)

    def moveAllTo(self, position):
        for motor in self.motors:
            motor.move(position)

    def moveAllToTarget(self):
        if self.is_real:
            for motor in self.motors:
                time.sleep(0.01)
                if not isinstance(motor.target, tuple) or len(motor.target) != 2:
                    # Set a default target if the motor target is not set correctly
                    motor.target = (motor.theta, 'P')  # Use the current position as a default target
                motor.move(motor.target)
        else:
            # Check for invalid targets and set a safe default
            for motor in self.motors:
                if not isinstance(motor.target, tuple) or len(motor.target) != 2:
                    motor.target = (motor.theta, 'P')  # Default to the current position if no valid target
            # Call CoppeliaSim API with valid motor targets
            self.sim.callScriptFunction('setJointAngles', self.motorMovePositionScriptHandle,
                                        [motor.handle for motor in self.motors],
                                        [motor.target[0] for motor in self.motors])  # Only the first element (angle) is needed

    def initHomePos(self):
        if self.is_real:
            self.arduino_serial.send_command("1")
            time.sleep(2)

    def readBatteryLevel(self):
        if self.is_real:
            self.arduino_serial.send_command("30")
            return self.arduino_serial.read_line()

    def shutdown(self):
        if self.is_real:
            self.arduino_serial.send_command("100")
            
    # Controls/Kinematics/Dynamics methods

    def updateRobotCoM(self):
        rightArm = self.updateRightArmCoM()
        leftArm = self.updateLeftArmCoM()
        torso = self.updateTorsoCoM()
        rightLeg = self.updateRightLegCoM()
        leftLeg = self.updateLeftLegCoM()
        chestMass = 618.15
        chest = [0, 71.83, 54.35]
        rightArmMass = 524.11
        leftArmMass = 524.11
        torsoMass = 434.67
        rightLegMass = 883.81
        leftLegMass = 889.11
        massSum = rightArmMass+leftArmMass+torsoMass+chestMass
        CoMx = rightArm[0] * rightArmMass + leftArm[0] * leftArmMass + torso[0]*torsoMass + chest[0]*chestMass + rightLeg[0]*rightLegMass + leftLeg[0]*leftLegMass
        CoMy = rightArm[1] * rightArmMass + leftArm[1] * leftArmMass + torso[1]*torsoMass + chest[1]*chestMass + rightLeg[1]*rightLegMass + leftLeg[1]*leftLegMass
        CoMz = rightArm[2] * rightArmMass + leftArm[2] * leftArmMass + torso[2]*torsoMass + chest[2]*chestMass + rightLeg[2]*rightLegMass + leftLeg[2]*leftLegMass
        self.CoM = [CoMx / massSum, CoMy / massSum, CoMz / massSum]
        return self.CoM

    def updateRightArmCoM(self):
        motorList = [self.motors[0], self.motors[1], self.motors[2], self.motors[3], self.motors[4]]
        linkList = [self.links[0], self.links[1], self.links[2], self.links[3], self.links[4]]
        return poe.calcLimbCoM(motorList, linkList)
    
    def updateLeftArmCoM(self):
        motorList = [self.motors[5], self.motors[6], self.motors[7], self.motors[8], self.motors[9]]
        linkList = [self.links[5], self.links[6], self.links[7], self.links[8], self.links[9]]
        return poe.calcLimbCoM(motorList, linkList)
    
    def updateTorsoCoM(self):
        motorList = [self.motors[11], self.motors[13], self.motors[10], self.motors[12], self.motors[14]]
        linkList = [self.links[11], self.links[13], self.links[10], self.links[12], self.links[14]]
        return poe.calcLimbCoM(motorList, linkList)
    
    def updateRightLegCoM(self):
        motorList = [self.motors[15], self.motors[16], self.motors[17], self.motors[18], self.motors[19]]
        linkList = [self.links[15], self.links[16], self.links[17], self.links[18], self.links[19]]
        #print(poe.calcLegCoM(self, motorList))
        return poe.calcLegCoM(self, motorList, linkList)

    def updateLeftLegCoM(self):
        motorList = [self.motors[20], self.motors[21], self.motors[22], self.motors[23], self.motors[24]]
        linkList = [self.links[20], self.links[21], self.links[22], self.links[23], self.links[24]]
        #print(poe.calcLegCoM(self, motorList))
        return poe.calcLegCoM(self, motorList, linkList)
      
    def locate(self, motor):
        slist = []
        thetaList = []
        M = motor.M
        slist.append(motor.twist)
        thetaList.append(motor.get_position())
        next = self.chain[motor.name]
        while next != "base":
            slist.append(next.twist)
            thetaList.append(next.get_position())
            next = self.chain[next.name]            
        slist.reverse()
        thetaList.reverse()
        # print(thetaList)
        location = mr.FKinSpace(M,np.transpose(slist),thetaList)
        return location
    
    def locatePolygon(self):
        slist = []
        thetaList = []
        locations = []
        rightAnkleM = [[1,0,0,-43.49],[0,1,0,659.84],[0,0,1,70.68],[1,0,0,0]]
        leftAnkleM = [[1,0,0,43.49],[0,1,0,659.84],[0,0,1,70.68],[1,0,0,0]]
        rightAnkleMotor = self.motors[Config.Joints.Right_Ankle_Joint.value]
        leftAnkleMotor = self.motors[Config.Joints.Left_Ankle_Joint.value]
        Ms = [rightAnkleM, leftAnkleM]
        ankleMotors = [rightAnkleMotor, leftAnkleMotor]
        for i in range(len(ankleMotors)):
            motor = ankleMotors[i]
            M = Ms[i]
            slist.append(motor.twist)
            thetaList.append(motor.get_position())
            next = self.chain[motor.name]
            while next != "base":
                slist.append(next.twist)
                thetaList.append(next.get_position())
                next = self.chain[next.name]         
            slist.reverse()
            thetaList.reverse()
            # print(thetaList)
            locations.append(mr.FKinSpace(M,np.transpose(slist),thetaList)[0:3,3])
        return locations
    
    def IK(self, motor, T, thetaGuess):
        """Computes the Inverse Kinematics from the Body Frame to the desired end effector motor

        Args:
            eeMotor (SimMotor): Motor you want to calculate IK towards
            T (4x4 Matrix): The desired final 4x4 matrix depicting the final position and orientation
        """
        Slist = []
        Slist.append(motor.twist)
        next = self.chain[motor.name]
        while next != "base":
            Slist.append(next.twist)
            next = self.chain[next.name]
        Slist.reverse()
        M = motor.home
        eomg = 0.01
        ev = 0.01
        return (mr.IKinSpace(Slist, M, T, thetaGuess, eomg, ev))

    # New method to get data from all IMUs
    def getAllIMUData(self):
        imu_data = {name: imu.getData() for name, imu in self.imus.items()}
        return imu_data

    def balanceAngle(self):
        balanceError = self.balancePoint - self.CoM
        
        # targetTheta = math.atan2(staticCoM[1] - staticKickLoc[1], staticCoM[2] - staticKickLoc[2])
        # kickMotorPos = self.locate(self.motors[Config.Joints.Left_Thigh_Kick_Joint.value])
        # currTheta = math.atan2(self.CoM[1] - kickMotorPos[1], self.CoM[2] - kickMotorPos[2])
        # thetaError = targetTheta - currTheta
        # self.PID.setError(thetaError)
        # newTarget = self.PID.calculate()

        self.PIDVel.setError(balanceError[2])
        newTarget = self.PIDVel.calculate()
        
        self.motors[10].target = (-0.000001*newTarget, 'V')
        
        # self.motors[22].target = (0.001*newTarget, 'V')
        # # self.motors[24].target = (-10, 'V')
        # self.motors[17].target = (-0.001*newTarget, 'V')
        # self.motors[19].target = (10, 'V')
        # self.IMUBalance(0, 0)
        return balanceError

    def balanceAngleOLD(self):
        # targetZ = 88
        # zError = targetZ - self.CoM[2]
        # target = 0.11
        # self.locate(self.motors[Config.Joints.Left_Ankle_Joint.value])*ankleL_to_sole
        staticCoM = [-9.2, -487.6, 90.5]
        staticKickLoc = [93.54, -209.39, 41.53]
        targetTheta = math.atan2(staticCoM[1] - staticKickLoc[1], staticCoM[2] - staticKickLoc[2])
        kickMotorPos = self.locate(self.motors[Config.Joints.Left_Thigh_Kick_Joint.value])
        currTheta = math.atan2(self.CoM[1] - kickMotorPos[1], self.CoM[2] - kickMotorPos[2])
        thetaError = targetTheta - currTheta
        # print(math.degrees(targetTheta), math.degrees(currTheta), thetaError, self.CoM)
        # print(self.CoM)
        # print(math.degrees(self.motors[22].target), math.degrees(self.motors[22].target), math.degrees(self.motors[17].target), math.degrees(self.motors[19].target))
        self.PID.setError(thetaError)
        newTarget = self.PID.calculate()
        
        self.motors[22].target = (-newTarget, 'P')
        self.motors[24].target = (-newTarget, 'P')
        self.motors[17].target = (newTarget, 'P')
        self.motors[19].target = (newTarget, 'P')
        self.IMUBalance(0, 0)
        return thetaError
    
    def decodeError(self, errorNum : int):
        errorNum = int(errorNum)
        errorDict = { 1: "Exceed Input Voltage Limit",
                      2: "Exceed Allow POT Limit",
                      4: "Exceed Temperature Limit",
                      8: "Invalid Packet (8)",
                      9: "Invalid Packet (9)",
                      16: "Overload Detected",
                      32: "Driver Fault Detected",
                      64: "EEP REG Distorted",
                      255: "No Communication"}
                      
        if(errorNum in errorDict.keys()):
            return errorDict[errorNum]
        else:
            return errorNum
    
    # If continuously called, checks motor statuses at the given interval (in seconds)
    def checkMotorsAtInterval(self, interval):
        if(time.time() > self.lastMotorCheck + interval):
            self.lastMotorCheck = time.time()
            self.checkMotors()

    # Checks the status of all motors
    def checkMotors(self):
        if(not self.is_real):
            return
        
        self.arduino_serial.send_command("50") # Check motors command

        while True:
            line = self.arduino_serial.read_line()
            if(line == "END" or line == None):
                return
            
            msg = line.split(" ")
            if(len(msg) >= 3):
                print(f"Error: {self.decodeError(msg[1])}, Motor: {msg[0]}, Angle: {msg[2]}")
            else:
                print(line)