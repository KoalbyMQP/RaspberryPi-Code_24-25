import numpy as np
import torch
import matplotlib.pyplot as plt
import time
import sys
from rl.env import BalanceEnv
from rl.agent import RLAgent

# File to save rewards
SAVE_FILE = "rewards.npy"

def plot_episode_rewards(rewards, window=50):
    """
    Plots the total episode rewards over iterations along with a moving average.
    
    Args:
        rewards (list or np.array): A list or array of total rewards per episode.
        window (int): The window size for computing the moving average. Default is 50.
    """
    plt.figure(figsize=(12, 6))
    episodes = np.arange(1, len(rewards) + 1)

    # Plot the raw episode rewards.
    plt.plot(episodes, rewards, label='Episode Reward', marker='o', linestyle='-', alpha=0.7)

    # Compute the moving average.
    if len(rewards) >= window:
        moving_avg = np.convolve(rewards, np.ones(window) / window, mode='valid')
        episodes_ma = np.arange(window, len(rewards) + 1)
        plt.plot(episodes_ma, moving_avg, label=f'Moving Average (window={window})', color='red', linewidth=2)

    plt.xlabel('Episode')
    plt.ylabel('Total Reward')
    plt.title('Episode Reward over Training Iterations')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def save_rewards(rewards):
    """
    Saves the reward history to a file.
    """
    np.save(SAVE_FILE, rewards)
    print(f"Saved rewards to {SAVE_FILE}")

def load_rewards():
    """
    Loads rewards from a file if available.
    """
    try:
        return np.load(SAVE_FILE).tolist()  # Convert back to list
    except FileNotFoundError:
        return []  # Start fresh if no previous data exists

def train(num_episodes=500):
    env = BalanceEnv(is_real=False)  
    state_dim = 3  
    action_dim = 3  
    agent = RLAgent(state_dim, action_dim, lr=1e-4, gamma=0.99)

    # Load previous rewards if available
    rewards_history = load_rewards()

    try:
        for episode in range(len(rewards_history), num_episodes):  # Continue from last saved episode
            state = env.reset()
            ep_reward = 0
            done = False
            while not done:
                action = agent.select_action(state)
                next_state, reward, done, _ = env.step(action)
                agent.rewards.append(reward)
                state = next_state
                ep_reward += reward
            loss = agent.finish_episode()
            rewards_history.append(ep_reward)
            print(f"Episode {episode+1}: Total Reward = {ep_reward:.3f}")

            # Save progress every 10 episodes
            if episode % 10 == 0:
                save_rewards(rewards_history)

    except KeyboardInterrupt:
        print("\nTraining interrupted. Saving progress...")
        save_rewards(rewards_history)  # Save rewards before exiting
        plot_episode_rewards(rewards_history, window=50)  # Show plot

    print("Training completed.")
    plot_episode_rewards(rewards_history, window=50)

if __name__ == "__main__":
    train()