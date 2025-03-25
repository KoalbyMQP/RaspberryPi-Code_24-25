# rl/env.py
import time
import numpy as np
import math
from backend.KoalbyHumanoid.Robot import Robot

ACTION_SCALE = 1.0
ACTION_LIMIT = math.radians(45)

# Define limits (in radians)
HIP_ACTION_LIMIT = math.radians(30)   # ±30° for hip adjustments
ARM_ACTION_LIMIT = math.radians(20)   # ±20° for arm adjustments

# Define base positions (in radians)
RIGHT_SHOULDER_BASE = 0.0                          # 0 (centered)
RIGHT_ABDUCTOR_BASE = math.radians(70)             # 70° downward (closer to body)
LEFT_SHOULDER_BASE  = 0.0                          # 0 (centered)
LEFT_ABDUCTOR_BASE  = math.radians(-70)            # -70° (closer to body)

# Scaling factors for mapping (tunable)
k1 = 0.2   # for shoulder (pitch-related)
k2 = 0.2   # for abductor (roll-related)

# Additional reward parameters (tunable)
DEVIATION_WEIGHT = 3.0        # Increase penalty on pitch/roll deviations
VELOCITY_PENALTY_WEIGHT = 0.5   # Penalty for rapid state changes (angular velocity)
ACTION_PENALTY_WEIGHT = 0.1     # Penalty for large motor commands
SURVIVAL_BONUS = 0.05           # Bonus per step for staying alive

class BalanceEnv:
    def __init__(self, is_real=False):
        # Create the robot instance (in simulation)
        self.robot = Robot(is_real)
        # Define maximum steps per episode and a falling threshold
        self.max_steps = 2000
        self.current_step = 0
        self.angle_threshold = math.radians(45)  # threshold for pitch or roll
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
        self.previous_state = state  # for smoothing and velocity estimation
        print(f"Fused data= {state}")
        return np.array(state)

    def step(self, action):
        """
        Apply an action (a 3-element vector) to the robot and return the new state,
        reward, done flag, and an (empty) info dictionary.
        """
        # Clip and scale hip actions
        hip_actions = np.clip(action[:3], -HIP_ACTION_LIMIT, HIP_ACTION_LIMIT)
        print(f"Action: {hip_actions}")

        scaled_action = hip_actions * ACTION_SCALE
        self.robot.motors[12].target = (scaled_action[2], 'P')
        self.robot.motors[13].target = (scaled_action[1], 'P')
        self.robot.motors[10].target = (-scaled_action[0], 'P')

        # Derive Arm Offsets from the Same Action
        right_shoulder_offset = k1 * scaled_action[1]
        left_shoulder_offset  = -k1 * scaled_action[1]  # flip sign for left arm
        
        if scaled_action[0] > 0:
            right_abductor_offset = k2 * scaled_action[0]
            left_abductor_offset = 0
        else:
            right_abductor_offset = 0
            left_abductor_offset  = -k2 * scaled_action[0]  # flip sign

        # Clip arm offsets to avoid excessive movement
        right_shoulder_offset = np.clip(right_shoulder_offset, -ARM_ACTION_LIMIT, ARM_ACTION_LIMIT)
        left_shoulder_offset  = np.clip(left_shoulder_offset, -ARM_ACTION_LIMIT, ARM_ACTION_LIMIT)
        right_abductor_offset = np.clip(right_abductor_offset, -ARM_ACTION_LIMIT, ARM_ACTION_LIMIT)
        left_abductor_offset  = np.clip(left_abductor_offset, -ARM_ACTION_LIMIT, ARM_ACTION_LIMIT)

        # Compute target positions for arms (offset added to base)
        right_shoulder_target = RIGHT_SHOULDER_BASE + right_shoulder_offset
        right_abductor_target = RIGHT_ABDUCTOR_BASE + right_abductor_offset
        left_shoulder_target  = LEFT_SHOULDER_BASE  + left_shoulder_offset
        left_abductor_target  = LEFT_ABDUCTOR_BASE  + left_abductor_offset

        # Set the arm motor targets:
        self.robot.motors[0].target = (right_shoulder_target, 'P')
        self.robot.motors[1].target = (right_abductor_target, 'P')
        self.robot.motors[5].target = (left_shoulder_target, 'P')
        self.robot.motors[6].target = (left_abductor_target, 'P')

        # Set leg motors to zero for stability
        for i in range(15, 25):
            self.robot.motors[i].target = (0, 'P')

        # Command all motors to move
        self.robot.moveAllToTarget()
        time.sleep(0.03)  # Reduced wait time for more frequent updates

        # Get the new state from fused IMU data
        imu_data = self.robot.imu_manager.getAllIMUData()
        right_chest = imu_data["RightChest"]
        left_chest = imu_data["LeftChest"]
        torso = imu_data["Torso"]
        fused = self.robot.fuse_imu_data(right_chest, left_chest, torso)
        raw_state = np.array(fused[:3])

        # Apply Smoothing (Rolling Average) and estimate angular velocity
        SMOOTHING_FACTOR = 0.85
        state = SMOOTHING_FACTOR * raw_state + (1 - SMOOTHING_FACTOR) * self.previous_state

        # Compute angular velocity (rate of change)
        angular_velocity = np.abs(state - self.previous_state)
        self.previous_state = state

        # --- Reward Calculation ---
        pitch, roll, yaw = state

        # Penalize deviations with increased weight
        deviation_penalty = DEVIATION_WEIGHT * (pitch**2 + roll**2) + 0.1 * yaw**2

        # Penalize rapid changes (angular velocity)
        velocity_penalty = VELOCITY_PENALTY_WEIGHT * np.sum(angular_velocity)

        # Penalize large hip actions
        hip_action_penalty = ACTION_PENALTY_WEIGHT * np.sum(np.abs(hip_actions))

        # Survival bonus for every step balanced
        reward = -deviation_penalty - velocity_penalty - hip_action_penalty + SURVIVAL_BONUS

        # Optionally, add a bonus for lasting longer (but not too high)
        reward += self.current_step * 0.0005

        # Inside your env.step() after obtaining the action (before clipping if possible)
        saturation_penalty = 0.1 * np.sum(np.abs(hip_actions) >= HIP_ACTION_LIMIT)
        reward = reward - saturation_penalty

        # --- Check termination conditions ---
        done = (abs(pitch) > self.angle_threshold or
                abs(roll) > self.angle_threshold or
                self.current_step >= self.max_steps)
        self.current_step += 1
        
        return np.array(state), reward, done, {}
    
def compute_reward(state, current_step, action):
    """
    (Optional alternative reward function)
    This function computes reward based on the state, step count, and actions.
    """
    SCALE_FACTOR = 1.0
    pitch, roll, yaw = state[:3] * SCALE_FACTOR
    action_penalty = np.sum(np.abs(action)) * 0.05

    good_threshold = math.radians(10)
    if abs(pitch) < good_threshold/2 and abs(roll) < good_threshold/2:
        bonus = 3.0
    elif abs(pitch) < good_threshold and abs(roll) < good_threshold:
        bonus = 1.5
    else:
        bonus = 0.0

    w_pitch = 2.0
    w_roll = 2.0
    w_yaw = 0.5

    reward = (
        bonus - (w_pitch * (pitch ** 3) + w_roll * (roll ** 3) + w_yaw * (yaw ** 3))
        - action_penalty
        + (current_step ** 2) * 0.2
    )
    return reward