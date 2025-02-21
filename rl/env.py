# rl/env.py
import time
import numpy as np
import math
from backend.KoalbyHumanoid.Robot import Robot

ACTION_SCALE = 1.0
ACTION_LIMIT = math.radians(45)

class BalanceEnv:
    def __init__(self, is_real=False):
        # Create the robot instance (in simulation)
        self.robot = Robot(is_real)
        # Define maximum steps per episode and a falling threshold
        self.max_steps = 2000
        self.current_step = 0
        self.angle_threshold = math.radians(45)  # ° threshold for pitch or roll
        self.reset()

    def reset(self):
        """Reset the robot to its initial (standing) pose and return the initial state."""
        
        self.robot.restart_simulation()
        self.robot.reset_position()  # set robot to a default standing pose
        time.sleep(0.5)  # allow time to stabilize
        self.current_step = 0

        # Get an initial state using fused IMU data (using three key IMUs)
        imu_data = self.robot.imu_manager.getAllIMUData()
        right_chest = imu_data["RightChest"]
        left_chest = imu_data["LeftChest"]
        torso = imu_data["Torso"]
        fused = self.robot.fuse_imu_data(right_chest, left_chest, torso)
        state = fused[:3]
        # state[2] = state[2]-(np.pi*2)
        self.previous_state = state
        print(f"Fused data= {state}")
        return np.array(state)

    def step(self, action):
        """
        Apply an action (a 3-element vector) to the robot and return the new state,
        reward, done flag, and an (empty) info dictionary.

        The action controls three motors:
         - motor[12]: yaw correction (set to action[2])
         - motor[13]: pitch correction (set to action[1])
         - motor[10]: roll correction (set to -action[0])
        """
        # Set the target positions for the balance joints.
        clipped_action = np.clip(action, -ACTION_LIMIT, ACTION_LIMIT)
        print(f"Action: {clipped_action}")

        action = clipped_action * ACTION_SCALE
        self.robot.motors[12].target = (action[2], 'P')
        self.robot.motors[13].target = (action[1], 'P')
        self.robot.motors[10].target = (-action[0], 'P')

        # (Optionally) keep arms and legs at a fixed, safe configuration:
        self.robot.motors[1].target = (math.radians(70), 'P')   # Right arm
        self.robot.motors[6].target = (math.radians(-70), 'P')  # Left arm
        for i in range(15, 25):  # set all leg motors to zero
            self.robot.motors[i].target = (0, 'P')

        # Command all motors to move
        self.robot.moveAllToTarget()
        time.sleep(0.05)  # wait a short time (50 ms) for simulation update

        # Get the new state from fused IMU data
        imu_data = self.robot.imu_manager.getAllIMUData()
        right_chest = imu_data["RightChest"]
        left_chest = imu_data["LeftChest"]
        torso = imu_data["Torso"]
        fused = self.robot.fuse_imu_data(right_chest, left_chest, torso)
        SCALE_FACTOR = 1.0
        normalized_state = np.array(fused[:3]) * SCALE_FACTOR

        # **Apply Smoothing (Rolling Average)**
        SMOOTHING_FACTOR = 0.9  
        state = SMOOTHING_FACTOR * np.array(fused[:3]) + (1 - SMOOTHING_FACTOR) * self.previous_state
        self.previous_state = state  # Store for next step

        reward = compute_reward(state, self.current_step, action)

        # End the episode if the pitch or roll is too large or if max steps reached
        done = (abs(state[0]) > self.angle_threshold or
                abs(state[1]) > self.angle_threshold or 
                abs(state[2]) > 2 * self.angle_threshold or 
                self.current_step >= self.max_steps)
        self.current_step += 1
        return np.array(state), reward, done, {}
    
def compute_reward(state, current_step, action):
    SCALE_FACTOR = 1.0
    pitch, roll, yaw = state[:3] * SCALE_FACTOR

    action_penalty = np.sum(np.abs(action)) * 0.05

    # Define thresholds for good balance
    good_threshold = math.radians(10)
    if abs(pitch) < good_threshold/2 and abs(roll) < good_threshold/2:
        bonus = 3.0  # reward bonus for being nearly perfect
    elif abs(pitch) < good_threshold and abs(roll) < good_threshold:
        bonus = 1.50  # reward bonus for being kinda good
    else:
        bonus = 0.0

    # Weigh yaw less if not critical
    w_pitch = 2.0
    w_roll = 2.0
    w_yaw = 0.5

    # Negative penalty for deviation, plus bonus if within threshold
    reward = (
        bonus - (w_pitch * (pitch ** 2) + w_roll * (roll ** 2) + w_yaw * (yaw ** 2))
        - action_penalty
        + (current_step ** 2) * 0.2
    )

    return reward