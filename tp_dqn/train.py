import argparse
import math
from datetime import datetime

import agent
import env
import tensorboardX as tbx


def create_directory_name(bandwidth, num_ue, rate):
    return f"logs/bw-{bandwidth}_ue-{num_ue}_rate-{rate}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"


def get_parser():
    parser = argparse.ArgumentParser(
        description="Train DQN agent for drone network",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-n", "--num_ue", type=int, default=-1, help="Number of UEs range(10,81,10)"
    )
    parser.add_argument("-r", "--rate", type=int, default=-1, help="Rate range(0,11)")
    return parser


def compute_pf(user_list, initial_data):
    pf = sum(
        [
            math.log(user.total_data - initial_data)
            for user in user_list
            if user.total_data != initial_data
        ]
    )
    return pf


def train(num_ue, rate, writer):
    drone_env = env.DroneDQNEnv(num_ue=num_ue, datarate_window=[rate, rate])
    state = drone_env.reset()
    input_dim = state.shape[0]
    drone_agent = agent.Agent(
        input_dim=input_dim,
        output_dim=7,
        lr=1e-4,
        gamma=0.99,
        tau=1e-3,
        epsilon=1,
        epsilon_decay=0.999,
        min_epsilon=0.00,
        soft_update_period=1,
        learning_period=5,
        batch_size=512,
        buffer_size=10000,
    )

    num_episodes = 5000
    avg_pf = 0
    avg_reward = 0
    print_period = 200
    for eps_idx in range(1, num_episodes + 1):
        if eps_idx % print_period == 0:
            print(
                f"Episode: {eps_idx}, PF: {avg_pf/print_period:.3f}, reward: {avg_reward/20/print_period:.3f}, loss: {drone_agent.latest_loss:.3f}, epsilon: {drone_agent.epsilon:.2f}"
            )
            drone_agent.save_network(
                f"checkpoints/bw-{env.db.BANDWIDTH_ORIG}/ue-{num_ue}_rate-{rate}.pt"
            )
            avg_pf = 0
            avg_reward = 0
        state = drone_env.reset()
        while True:
            action = drone_agent.act(state)
            next_state, reward, done, _ = drone_env.step(action)
            drone_agent.step(state, action, reward, next_state, done)
            state = next_state
            avg_reward += reward
            if done:
                break
        pf = compute_pf(drone_env.current_node.user_list, drone_env.initial_data)
        avg_pf += pf
        writer.add_scalar("PF", pf, eps_idx)
        writer.add_scalar("Reward", avg_reward, eps_idx)
        writer.add_scalar("Loss", drone_agent.latest_loss, eps_idx)


def main():
    parser = get_parser()
    args = parser.parse_args()

    bandwidth = env.db.BANDWIDTH_ORIG
    num_ue = args.num_ue if args.num_ue != -1 else 20
    rate = args.rate if args.rate != -1 else 5

    # Initialize TensorBoardX SummaryWriter with dynamic directory name
    log_dir = create_directory_name(bandwidth, num_ue, rate)
    writer = tbx.SummaryWriter(log_dir)

    if args.num_ue != -1:
        train(num_ue=args.num_ue, rate=5, writer=writer)
    if args.rate != -1:
        train(num_ue=20, rate=args.rate, writer=writer)


if __name__ == "__main__":
    main()
