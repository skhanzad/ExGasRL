from src.models import LunarLandingModel
from src.agents import LunarLandingAgent
import gymnasium as gym


def main():
    # print(name for name, _ in gym.envs.registry.items())
    # Initialize the agent and model
    agent = LunarLandingAgent()
    model = LunarLandingModel(input_size=8, hidden_size=64, output_size=4)

    # Train the agent
    agent.train(episodes=1000)

    # Run the agent in the environment
    agent.run()

    # Close the environment
    agent.close()


if __name__ == "__main__":
    main()