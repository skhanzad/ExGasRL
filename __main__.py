from src.agents import BlackjackAgent, gym



def main():    # Example usage of the BlackjackAgent
    env = gym.make("Blackjack-v1")
    agent = BlackjackAgent(
        env,
        learning_rate=0.1,
        initial_epsilon=1.0,
        epsilon_decay=0.99,
        final_epsilon=0.1,
    )

    obs, _ = env.reset()
    done = False

    while not done:
        action = agent.get_action(obs)
        obs, reward, done, _, _ = env.step(action)
        agent.update(obs, action, reward, done, None)


    print(f"final Q-values: {agent.q_values}")
    print("Training complete!")


if __name__ == "__main__":
    main()