import argparse
import math
from datetime import datetime

import env
import msd3qn_agent as agent
import tensorboardX as tbx


def create_directory_name(bandwidth, num_ue, rate):
    return f"logs/bw-{bandwidth}_ue-{num_ue}_rate-{rate}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"


def get_parser():
    parser = argparse.ArgumentParser(
        description="Train DQN agent for drone network",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-m",
        "--mode",
        type=str,
        default="",
        choices=["user", "rate"],
    )
    parser.add_argument(
        "-r",
        "--rate",
        type=float,
    )
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
    print(
        f"-------- Training start: num_ue={num_ue}, rate={rate}, bw={env.db.BANDWIDTH_ORIG} --------"
    )
    drone_env = env.DroneDQNEnv(num_ue=num_ue, datarate_window=[rate, rate])
    state = drone_env.reset()
    input_dim = state.shape[0]
    drone_agent = agent.Agent(
        input_dim=input_dim,
        output_dim=7,
        lr=1e-4,
        gamma=0.0,
        tau=1e-3,
        epsilon=1,
        epsilon_decay=0.995,
        min_epsilon=0.00,
        soft_update_period=1,
        learning_period=5,
        batch_size=256,
        buffer_size=10000,
        n_step=3,
    )

    best_reward = -math.inf
    num_episodes = 5000
    avg_pf = 0
    avg_reward = 0
    print_period = 200
    for eps_idx in range(1, num_episodes + 1):
        if eps_idx % print_period == 0:
            print(
                f"Episode: {eps_idx}, PF: {avg_pf/print_period:.3f}, reward: {avg_reward/20/print_period:.3f}, loss: {drone_agent.latest_loss:.3f}, epsilon: {drone_agent.epsilon:.2f}"
            )
            if avg_reward > best_reward:
                best_reward = avg_reward
                drone_agent.save_network(
                    f"checkpoints_msd3qn/bw-{env.db.BANDWIDTH_ORIG}/ue-{num_ue}_rate-{rate}.pt"
                )
            avg_pf = 0
            avg_reward = 0
        state = drone_env.reset()
        eps_reward = 0
        while True:
            action = drone_agent.act(state)
            next_state, reward, done, _ = drone_env.step(action)
            drone_agent.step(state, action, reward, next_state, done)
            state = next_state
            eps_reward += reward
            if done:
                break
        pf = compute_pf(drone_env.current_node.user_list, drone_env.initial_data)
        avg_pf += pf
        avg_reward += eps_reward
        writer.add_scalar("PF", pf, eps_idx)
        writer.add_scalar("Reward", eps_reward, eps_idx)
        writer.add_scalar("Loss", drone_agent.latest_loss, eps_idx)


def main():
    parser = get_parser()
    args = parser.parse_args()

    bandwidth = env.db.BANDWIDTH_ORIG

    # Initialize TensorBoardX SummaryWriter with dynamic directory name

    if args.mode == "user":
        rate = 5
        for num_ue in range(10, 81, 10):
            log_dir = create_directory_name(bandwidth, num_ue, rate)
            writer = tbx.SummaryWriter(log_dir)
            train(num_ue=num_ue, rate=rate, writer=writer)
    elif args.mode == "rate":
        num_ue = 20
        log_dir = create_directory_name(bandwidth, num_ue, args.rate)
        writer = tbx.SummaryWriter(log_dir)
        train(num_ue=num_ue, rate=args.rate, writer=writer)


if __name__ == "__main__":
    main()
