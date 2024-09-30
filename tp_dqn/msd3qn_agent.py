import os
import random
from collections import deque, namedtuple

import env
import numpy as np
import torch
from torch import nn


class MSD3QNetwork(nn.Module):
    def __init__(self, input_size, output_size, hidden_size=1024, num_layers=16):
        super(MSD3QNetwork, self).__init__()
        self.layers = nn.ModuleList()

        # Add the input layer
        self.layers.append(nn.Linear(input_size, hidden_size))

        for _ in range(num_layers - 2):
            # Add linear layer
            self.layers.append(nn.Linear(hidden_size, hidden_size))
        self.layers.append(nn.Linear(hidden_size, 128))
        self.batch_normalizations = nn.ModuleList(
            [nn.BatchNorm1d(layer.out_features) for layer in self.layers]
        )

        # Add the output layer
        self.value_layer = nn.Linear(128, 1)
        self.advantage_layer = nn.Linear(128, output_size)

    def forward(self, x):
        for layer, bn in zip(self.layers, self.batch_normalizations):
            residual = x
            x = bn(layer(x))

            if layer != self.layers[0] and layer != self.layers[-1]:
                x = nn.functional.relu(x) + residual
            else:
                x = nn.functional.relu(x)

        # Calculate value and advantage
        value = self.value_layer(x)  # V(s)
        advantage = self.advantage_layer(x)  # A(s, a)

        # Combine value and advantage into Q-values
        Q_values = value + (advantage - advantage.mean())
        return Q_values


class Agent:
    def __init__(
        self,
        input_dim,
        output_dim,
        lr,
        gamma,
        tau,
        epsilon,
        epsilon_decay,
        min_epsilon=0.01,
        soft_update_period=1,
        buffer_size=10000,
        batch_size=4096,
        seed=0,
        learning_period=40,
        n_step=5,
        warm_start=5000,
    ):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        # Env variables
        self.env = env.DroneDQNEnv()
        self.epsilon = epsilon
        self.min_epsilon = min_epsilon
        self.epsilon_decay = epsilon_decay

        # Network variables
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.num_action = output_dim
        self.target_q_network = MSD3QNetwork(input_dim, output_dim).to(self.device)
        self.local_q_network = MSD3QNetwork(input_dim, output_dim).to(self.device)
        self.update_count = 0
        self.soft_update_period = soft_update_period

        # Learning variables
        self.lr = lr
        self.gamma = gamma
        self.tau = tau
        self.step_count = 0
        self.learning_period = learning_period
        self.optimizer = torch.optim.Adam(self.local_q_network.parameters(), lr=lr)
        self.scheduler = torch.optim.lr_scheduler.StepLR(
            self.optimizer, step_size=500, gamma=0.9
        )
        self.criterion = nn.MSELoss()
        self.warm_start = warm_start
        # self.criterion = nn.SmoothL1Loss()

        self.latest_loss = -1
        self.n_step = n_step
        self.memory = ReplayBuffer(
            buffer_size=buffer_size,
            batch_size=batch_size,
            n_step=n_step,
            gamma=gamma,
            seed=seed,
        )

    def load_network(self, path):
        self.target_q_network.load_state_dict(torch.load(path))
        self.local_q_network.load_state_dict(torch.load(path))

    def save_network(self, path):
        if not os.path.exists(os.path.dirname(path)):
            os.makedirs(os.path.dirname(path))
        torch.save(self.target_q_network.state_dict(), path)

    def soft_update(self, target, source):
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(
                target_param.data * (1.0 - self.tau) + param.data * self.tau
            )

    def step(self, state, action, reward, next_state, done):
        reward = reward / self.n_step
        self.memory.add(state, action, reward, next_state, done)

        self.step_count += 1
        if self.step_count % self.learning_period == 0:
            if len(self.memory) > self.memory.batch_size:
                experiences = self.memory.sample()
                self.learn(experiences)
                if self.step_count > self.warm_start:
                    self.epsilon = max(
                        self.min_epsilon, self.epsilon * self.epsilon_decay
                    )

    def act(self, state: torch.Tensor):
        """Returns actions for given state as per current policy.

        Params
        ======
            state (array_like): current state
        """
        state = state.to(self.device)
        self.local_q_network.eval()
        with torch.no_grad():
            action_values = self.local_q_network(state.unsqueeze(dim=0))
        self.local_q_network.train()

        # Epsilon-greedy action selection
        if random.random() > self.epsilon:
            return np.argmax(action_values.cpu().data.numpy())
        else:
            return random.choice(np.arange(self.num_action))

    def learn(self, experiences):
        """Update value parameters using given batch of experience tuples.

        Params
        ======
            experiences (Tuple[torch.Tensor]): tuple of (s, a, r, s', done) tuples
            gamma (float): discount factor
        """

        states, actions, rewards, next_states, dones = experiences

        # Get max predicted Q values (for next states) from target model
        Q_targets_next = (
            self.target_q_network(next_states).detach().max(1)[0].unsqueeze(1)
        )
        # Compute Q targets for current states
        Q_targets = rewards + (self.gamma * Q_targets_next * (1 - dones))

        # Get expected Q values from local model
        Q_expected = self.local_q_network(states).gather(1, actions)

        # Compute loss
        loss = self.criterion(Q_expected, Q_targets)
        self.latest_loss = loss.item()
        # Minimize the loss
        self.optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.local_q_network.parameters(), 1)
        self.optimizer.step()
        self.scheduler.step()

        # ------------------- update target network ------------------- #
        self.update_count += 1
        if self.update_count % self.soft_update_period == 0:
            self.soft_update(self.target_q_network, self.local_q_network)


class ReplayBuffer:
    """Fixed-size buffer to store experience tuples."""

    def __init__(self, buffer_size, batch_size, n_step, gamma, seed=0):
        """Initialize a ReplayBuffer object.

        Params
        ======
            buffer_size (int): maximum size of buffer
            batch_size (int): size of each training batch
            n_step (int): multi-step
            gamma: discount factor
            seed (int): random seed
        """
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.memory = deque(maxlen=buffer_size)
        self.batch_size = batch_size
        self.n_step = n_step
        self.gamma = gamma
        self.experience = namedtuple(
            "Experience",
            field_names=["state", "action", "reward", "next_state", "done"],
        )
        self.n_step_buffer = deque(maxlen=n_step)
        self.seed = random.seed(seed)

    def add(self, state, action, reward, next_state, done):
        """Add a new experience to memory."""
        self.n_step_buffer.append((state, action, reward, next_state, done))

        if len(self.n_step_buffer) == self.n_step:
            state, action, reward, next_state, done = self.calc_multistep_return(
                self.n_step_buffer
            )
            e = self.experience(state, action, reward, next_state, done)
            self.memory.append(e)

    def calc_multistep_return(self, n_step_buffer):
        Return = 0
        for i in range(self.n_step):
            Return += self.gamma**i * n_step_buffer[i][2]
        return (
            n_step_buffer[0][0],
            n_step_buffer[0][1],
            Return,
            n_step_buffer[-1][3],
            n_step_buffer[-1][4],
        )

    def sample(self):
        """Randomly sample a batch of experiences from memory."""
        experiences = random.sample(self.memory, k=self.batch_size)

        states = (
            torch.from_numpy(np.vstack([e.state for e in experiences if e is not None]))
            .float()
            .to(self.device)
        )
        actions = (
            torch.from_numpy(
                np.vstack([e.action for e in experiences if e is not None])
            )
            .long()
            .to(self.device)
        )
        rewards = (
            torch.from_numpy(
                np.vstack([e.reward for e in experiences if e is not None])
            )
            .float()
            .to(self.device)
        )
        next_states = (
            torch.from_numpy(
                np.vstack([e.next_state for e in experiences if e is not None])
            )
            .float()
            .to(self.device)
        )
        dones = (
            torch.from_numpy(
                np.vstack([e.done for e in experiences if e is not None]).astype(
                    np.uint8
                )
            )
            .float()
            .to(self.device)
        )

        return (states, actions, rewards, next_states, dones)

    def __len__(self):
        """Return the current size of internal memory."""
        return len(self.memory)


def main():
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    device = torch.device("cpu")
    import time

    avg_time = 0
    for i in range(100):
        start = time.time()
        agent = Agent(
            input_dim=3,
            output_dim=3,
            lr=1e-3,
            gamma=0.99,
            tau=1e-3,
            epsilon=1.0,
            epsilon_decay=0.995,
        )
        avg_time += time.time() - start
    print(avg_time / 100)

    print(agent.act(torch.Tensor([1, 2, 3]).to(device)))


if __name__ == "__main__":
    main()
