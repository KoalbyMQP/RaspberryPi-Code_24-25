import numpy as np
import torch
import matplotlib.pyplot as plt
import time
import sys
from rl.env import BalanceEnv
from rl.ppo_agent import PPOAgent  # Updated to use PPO!

# File to save rewards
SAVE_FILE = "rewards.npy"

def plot_episode_rewards(rewards, window=50):
    """
    Plots the total episode rewards over iterations along with a moving average.
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
    """ Saves the reward history to a file. """
    np.save(SAVE_FILE, rewards)
    print(f"Saved rewards to {SAVE_FILE}")

def load_rewards():
    """ Loads rewards from a file if available. """
    try:
        return np.load(SAVE_FILE).tolist()
    except FileNotFoundError:
        return []  # Start fresh if no previous data exists

def train(num_episodes=500):
    env = BalanceEnv(is_real=False)  
    state_dim = 3  
    action_dim = 3  
    agent = PPOAgent(state_dim, action_dim, lr=3e-4, gamma=0.99, clip_epsilon=0.2, epochs=10, batch_size=32)  # Using PPO

    rewards_history = load_rewards()  # Load previous rewards

    try:
        for episode in range(len(rewards_history), num_episodes):  # Continue from last saved episode
            state = env.reset()
            ep_reward = 0
            done = False

            while not done:
                action = agent.select_action(state)
                next_state, reward, done, _ = env.step(action)

                agent.rewards.append(reward)  # Store reward
                state = next_state
                ep_reward += reward

                # Store experience for PPO
                agent.store_experience(state, action, reward)

            loss = agent.update_policy()  # PPO update

            rewards_history.append(ep_reward)
            print(f"Episode {episode+1}: Total Reward = {ep_reward:.3f}, Loss = {loss:.4f}")

            if episode % 10 == 0:
                save_rewards(rewards_history)  # Save every 10 episodes

    except KeyboardInterrupt:
        print("\nTraining interrupted. Saving progress...")
        save_rewards(rewards_history)  # Save rewards before exiting
        plot_episode_rewards(rewards_history, window=50)

    print("Training completed.")
    plot_episode_rewards(rewards_history, window=50)

if __name__ == "__main__":
    train()