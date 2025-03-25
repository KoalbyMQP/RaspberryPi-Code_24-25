import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np

# We'll use HIP_ACTION_LIMIT (≈0.5236 rad) as our maximum action
MAX_ACTION = 0.5235988

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_size=64):
        super(ActorCritic, self).__init__()

        self.fc1 = nn.Linear(state_dim, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)

        # Actor (Policy)
        # Instead of a plain linear output, we use tanh to bound the output.
        self.mean_linear = nn.Linear(hidden_size, action_dim)
        self.log_std = nn.Parameter(torch.zeros(action_dim))  # Learnable std

        # Critic (Value Function)
        self.value = nn.Linear(hidden_size, 1)

    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        
        # Compute raw mean then apply tanh and scale by MAX_ACTION.
        raw_mean = self.mean_linear(x)
        mean = torch.tanh(raw_mean) * MAX_ACTION

        # Clamp log_std before exponentiating to avoid extreme values.
        std = torch.exp(torch.clamp(self.log_std.clone(), min=-20, max=2))
        value = self.value(x)
        return mean, std, value

class PPOAgent:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, clip_epsilon=0.2, epochs=10, batch_size=32):
        self.policy = ActorCritic(state_dim, action_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)

        self.gamma = gamma
        self.clip_epsilon = clip_epsilon
        self.epochs = epochs
        self.batch_size = batch_size

        self.rewards = []
        self.saved_log_probs = []
        self.values = []
        self.states = []
        self.actions = []

    def select_action(self, state):
        state = torch.from_numpy(state).float().unsqueeze(0)
        mean, std, value = self.policy(state)

        dist = torch.distributions.Normal(mean, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum()

        self.saved_log_probs.append(log_prob.unsqueeze(0))  # Store as a 1D tensor
        self.values.append(value)
        self.states.append(state)
        self.actions.append(action)

        return action.detach().numpy().squeeze()

    def store_experience(self, state, action, reward):
        """
        Store experience for PPO update
        """
        self.rewards.append(reward)

    def compute_returns(self):
        R = 0
        returns = []
        advantages = []
        for r, value in zip(reversed(self.rewards), reversed(self.values)):
            R = r + self.gamma * R
            advantage = R - value.item()
            returns.insert(0, R)
            advantages.insert(0, advantage)

        returns = torch.tensor(returns).float()
        advantages = torch.tensor(advantages).float()

        # Avoid division by a near-zero standard deviation in advantage normalization
        if advantages.std() > 1e-6:
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-5)

        return returns, advantages

    def update_policy(self):
        if len(self.saved_log_probs) == 0:
            print("Skipping update: No actions stored in this episode.")
            return 0  

        returns, advantages = self.compute_returns()
        states = torch.cat(self.states)
        actions = torch.cat(self.actions).clone().detach()
        old_log_probs = torch.cat(self.saved_log_probs).clone().detach()

        for _ in range(self.epochs):
            mean, std, values = self.policy(states)
            dist = torch.distributions.Normal(mean, std)

            new_log_probs = dist.log_prob(actions).sum(dim=-1)
            ratio = torch.exp(new_log_probs - old_log_probs)

            # Compute Clipped Surrogate Loss
            clipped_adv = torch.clamp(ratio, 1 - self.clip_epsilon, 1 + self.clip_epsilon) * advantages
            policy_loss = -torch.min(ratio * advantages, clipped_adv).mean()
            value_loss = nn.functional.mse_loss(values.squeeze(), returns)

            # Total loss
            loss = policy_loss + 0.5 * value_loss

            # Gradient Clipping
            self.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_norm=0.5)
            self.optimizer.step()

        # Clear storage
        self.saved_log_probs.clear()
        self.rewards.clear()
        self.values.clear()
        self.states.clear()
        self.actions.clear()

        return loss