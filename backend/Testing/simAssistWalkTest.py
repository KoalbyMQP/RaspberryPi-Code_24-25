import sys, time, math

import numpy as np 
sys.path.append("./")
from backend.KoalbyHumanoid.Robot import Robot
from backend.KoalbyHumanoid.trajPlannerPose import TrajPlannerPose
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
from backend.KoalbyHumanoid.Config import Joints
import matplotlib.pyplot as plt
from backend.KoalbyHumanoid.Plotter import Plotter

# Edit to declare if you are testing the sim or the real robot
is_real = False

robot = Robot(is_real)

print("Setup Complete")

plotter = Plotter(10, False)

robot.motors[1].target = (math.radians(80), 'P')
robot.motors[6].target = (math.radians(-80), 'P')


robot.motors[17].target = (math.radians(20), 'P')
robot.motors[18].target = (math.radians(-40), 'P')
robot.motors[19].target = (math.radians(-20), 'P')
robot.motors[22].target = (math.radians(-20), 'P')
robot.motors[23].target = (math.radians(40), 'P')
robot.motors[24].target = (math.radians(20), 'P')
robot.motors[3].target = (math.radians(90), 'P')
robot.motors[8].target = (math.radians(-90), 'P')
robot.motors[14].target = (0, 'P')

prevTime = time.time()
simStartTime = time.time()

while time.time() - simStartTime < 1:
    time.sleep(0.01)
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()



right_setPointTimes = [[0,0,0], [2,2,2], [4,4,4], [6,6,6], [8,8,8]]
right_angles = [[math.radians(20),math.radians(-40), math.radians(-20)], 
                [math.radians(27.8416),math.radians(-54.0287), math.radians(-26.1871)], 
                [math.radians(36.9495),math.radians(-56.3311), math.radians(-19.3816)], 
                [math.radians(39.7712), math.radians(-48.2004), math.radians(-12.4292)], 
                [math.radians(30.1578),math.radians(-32.3075), math.radians(-2.1477)]]
right_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
right_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

left_setPointTimes = [[0,0,0],[8,8,8]]
left_angles = [[math.radians(-20),math.radians(40), math.radians(20)],
               [math.radians(-5.9552),math.radians(32.3075), math.radians(26.3523)]]
left_vels = [[0,0,0],[0,0,0],[0,0,0]]
left_accels = [[0,0,0],[0,0,0],[0,0,0]]

right_tj = TrajPlannerTime(right_setPointTimes, right_angles, right_vels, right_accels)
left_tj = TrajPlannerTime(left_setPointTimes, left_angles, left_vels, left_accels)


state = 0
startTime = time.time()
print("State = 0")
while True:
    right_points = right_tj.getQuinticPositions(time.time() - startTime)
    left_points = left_tj.getQuinticPositions(time.time() - startTime)
    robot.motors[17].target = (right_points[0], 'P')
    robot.motors[18].target = (right_points[1], 'P')
    robot.motors[19].target = (right_points[2], 'P')
    robot.motors[22].target = (left_points[0], 'P')
    robot.motors[23].target = (left_points[1], 'P')
    robot.motors[24].target = (left_points[2], 'P')
    robot.IMUBalance(0,0)
    robot.moveAllToTarget()
    match(state):
        case 0: 
            if(time.time() - startTime >= 8):
                ##ANGLES TO GO RIGHT TO LEFT
                right_setPointTimes = [[0,0,0], [2,2,2], [4,4,4], [6,6,6], [8,8,8]]
                right_angles =  [[math.radians(30.1578),math.radians(-32.3075), math.radians(-2.1477)], 
                                [math.radians(26.8052),math.radians(-39.0804), math.radians(-12.2751)], 
                                [math.radians(20),math.radians(-40), math.radians(-20)], 
                                [math.radians(10.1913), math.radians(-35.4586), math.radians(-25.2674)], 
                                [math.radians(-3.8532),math.radians(-22.8189), math.radians(-25.6721)]]
                right_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                right_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

                left_setPointTimes = [[0,0,0], [2,2,2], [4,4,4], [6,6,6], [8,8,8]]
                left_angles = [[math.radians(-5.9552),math.radians(32.3075), math.radians(26.3525)], 
                                [math.radians(-14.3033),math.radians(48.2004), math.radians(33.8971)], 
                                [math.radians(-42.5117),math.radians(70.3060), math.radians(27.7943)], 
                                [math.radians(-40.9558), math.radians(42.0062), math.radians(4.0504)], 
                                [math.radians(-29.3238),math.radians(22.8186), math.radians(-4.5052)]]
                left_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                left_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

                right_tj = TrajPlannerTime(right_setPointTimes, right_angles, right_vels, right_accels)
                left_tj = TrajPlannerTime(left_setPointTimes, left_angles, left_vels, left_accels)
                startTime = time.time()
                print("State = 1")
                state = 1
            
        case 1: ##Right to Left
            if(time.time() - startTime >= 8):
                ##ANGLES TO GO LEFT TO RIGHT
                right_setPointTimes = [[0,0,0], [2,2,2], [4,4,4], [6,6,6], [8,8,8]]
                right_angles =  [[math.radians(-3.8532),math.radians(-22.8189), math.radians(-26.6721)], 
                                [math.radians(6.0931),math.radians(-42.0062), math.radians(-35.9131)], 
                                [math.radians(37.0296),math.radians(-70.3060), math.radians(-33.2765)], 
                                [math.radians(39.7712), math.radians(-48.2004), math.radians(-8.4292)], 
                                [math.radians(30.1578),math.radians(-32.3075), math.radians(-2.1497)]]
                right_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                right_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

                left_setPointTimes = [[0,0,0], [2,2,2], [4,4,4], [6,6,6], [8,8,8]]
                left_angles = [[math.radians(-29.3238),math.radians(22.8186), math.radians(-4.5052)], 
                                [math.radians(-29.4673),math.radians(35.4585), math.radians(5.9912)], 
                                [math.radians(-20),math.radians(40), math.radians(20)], 
                                [math.radians(-16.9371), math.radians(39.0803), math.radians(22.1432)], 
                                [math.radians(-5.9552),math.radians(32.3075), math.radians(26.3523)]]
                left_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                left_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                right_tj = TrajPlannerTime(right_setPointTimes, right_angles, right_vels, right_accels)
                left_tj = TrajPlannerTime(left_setPointTimes, left_angles, left_vels, left_accels)
                print("State = 2")
                startTime = time.time()
                state = 0
        case 2: 
            if(time.time() - startTime >= 8):
                ##ANGLES TO GO FROM RIGHT TO LEFT
                right_setPointTimes = [[0,0,0], [2,2,2], [4,4,4], [6,6,6], [8,8,8]]
                right_angles =  [[math.radians(30.1578),math.radians(-32.3075), math.radians(-2.1477)], 
                                [math.radians(26.8052),math.radians(-39.0804), math.radians(-12.2751)], 
                                [math.radians(20),math.radians(-40), math.radians(-20)], 
                                [math.radians(10.1913), math.radians(-35.4586), math.radians(-25.2674)], 
                                [math.radians(-3.8532),math.radians(-22.8189), math.radians(-25.6721)]]
                right_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                right_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

                left_setPointTimes = [[0,0,0], [2,2,2], [4,4,4], [6,6,6], [8,8,8]]
                left_angles = [[math.radians(-5.9552),math.radians(32.3075), math.radians(26.3525)], 
                                [math.radians(-14.3033),math.radians(48.2004), math.radians(33.8971)], 
                                [math.radians(-42.5117),math.radians(70.3060), math.radians(27.7943)], 
                                [math.radians(-40.9558), math.radians(42.0062), math.radians(4.0504)], 
                                [math.radians(-29.3238),math.radians(22.8186), math.radians(-4.5052)]]
                left_vels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
                left_accels = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

                right_tj = TrajPlannerTime(right_setPointTimes, right_angles, right_vels, right_accels)
                left_tj = TrajPlannerTime(left_setPointTimes, left_angles, left_vels, left_accels)
                startTime = time.time()
                print("State = 1")
                state = 1