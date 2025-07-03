from collections import defaultdict
import gymnasium as gym
import numpy as np


class LunarLandingAgent:
    def __init__(self, env_name='LunarLander-v3', render_mode=None):
        self.env = gym.make(env_name)  # Non-rendering env for training
        self.eval_env = gym.make(env_name, render_mode=render_mode) if render_mode else None

        self.q_table = defaultdict(lambda: np.zeros(self.env.action_space.n))  # type: ignore
        self.alpha = 0.1  # Learning rate
        self.gamma = 0.99  # Discount factor
        self.epsilon = 0.1  # Exploration rate

        self.bins = np.linspace(-1, 1, 20)  # Example bins for discretization

    def discretize(self, state):
        """Discretize continuous state into bins for Q-table lookup."""
        return tuple(np.digitize(s, self.bins) for s in state)

    def choose_action(self, state):
        state_key = self.discretize(state)
        if np.random.rand() < self.epsilon:
            return self.env.action_space.sample()  # Explore
        else:
            return np.argmax(self.q_table[state_key])  # Exploit

    def update_q_value(self, state, action, reward, next_state):
        state_key = self.discretize(state)
        next_state_key = self.discretize(next_state)

        best_next_action = np.argmax(self.q_table[next_state_key])
        td_target = reward + self.gamma * self.q_table[next_state_key][best_next_action]
        td_error = td_target - self.q_table[state_key][action]

        self.q_table[state_key][action] += self.alpha * td_error

    def train(self, episodes=1000):
        for episode in range(episodes):
            state, _ = self.env.reset()
            done = False
            while not done:
                action = self.choose_action(state)
                next_state, reward, terminated, truncated, _ = self.env.step(action)
                done = terminated or truncated
                self.update_q_value(state, action, reward, next_state)
                state = next_state
            if episode % 100 == 0:
                print(f"Episode {episode}")

    def run(self, episodes=5):
        if not self.eval_env:
            self.eval_env = gym.make('LunarLander-v3', render_mode='human')

        for episode in range(episodes):
            state, _ = self.eval_env.reset()
            done = False
            total_reward = 0.0

            while not done:
                state_key = self.discretize(state)
                action = np.argmax(self.q_table[state_key])
                next_state, reward, terminated, truncated, _ = self.eval_env.step(action)
                done = terminated or truncated
                total_reward += float(reward)
                state = next_state

            print(f"Episode {episode + 1}, Total Reward: {total_reward}")

    def close(self):
        self.env.close()
        if self.eval_env:
            self.eval_env.close()
