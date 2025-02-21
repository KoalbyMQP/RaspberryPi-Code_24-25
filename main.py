# train_rl.py
import numpy as np
import torch
from rl.env import BalanceEnv
from rl.agent import RLAgent
import time

def train(num_episodes=500):
    env = BalanceEnv(is_real=False)  # simulation mode
    state_dim = 3   # using fused IMU angles: [pitch, roll, yaw]
    action_dim = 3  # actions for three torso motors
    agent = RLAgent(state_dim, action_dim, lr=1e-3, gamma=0.99)

    for episode in range(num_episodes):
        state = env.reset()
        ep_reward = 0
        done = False
        while not done:
            action = agent.select_action(state)
            next_state, reward, done, _ = env.step(action)
            agent.rewards.append(reward)
            state = next_state
            ep_reward += reward
        agent.finish_episode()
        print(f"Episode {episode+1}: Total Reward = {ep_reward:.3f}")
    print("Training completed.")

if __name__ == "__main__":
    train()