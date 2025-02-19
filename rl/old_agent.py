# rl/agent.py
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np

class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_size=64):
        super(PolicyNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.mean = nn.Linear(hidden_size, action_dim)
        # Learnable log standard deviation for each action dimension
        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        mean = self.mean(x)
        std = torch.exp(self.log_std)
        return mean, std

class RLAgent:
    def __init__(self, state_dim, action_dim, lr=1e-4, gamma=0.99):
        self.policy = PolicyNetwork(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        self.gamma = gamma
        self.saved_log_probs = []
        self.rewards = []

    def select_action(self, state):
        """
        Given a state (numpy array), sample an action from the policy distribution.
        Save the log probability for use in the policy update.
        """
        state = torch.from_numpy(state).float().unsqueeze(0)
        mean, std = self.policy(state)
        dist = torch.distributions.Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum()
        self.saved_log_probs.append(log_prob)
        # Return as a 1D numpy array (action vector)
        return action.detach().numpy().squeeze()

    def finish_episode(self):
        """
        Compute the discounted return and update the policy using the REINFORCE rule.
        """
        R = 0
        returns = []
        # Compute discounted returns (from the end of the episode backward)
        for r in self.rewards[::-1]:
            R = r + self.gamma * R
            returns.insert(0, R)
        returns = torch.tensor(returns)
        # Normalize returns for more stable training
        if returns.numel() > 1 and returns.std() > 0:
            returns = (returns - returns.mean()) / (returns.std() + 1e-5)
        else:
            # With a single element, simply subtract the mean (which yields 0)
            returns = returns - returns.mean()
        loss = 0
        for log_prob, R in zip(self.saved_log_probs, returns):
            loss -= log_prob * R
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        # Clear the saved rewards and log probabilities
        del self.rewards[:]
        del self.saved_log_probs[:]

        return loss