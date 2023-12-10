import agent
import env


def get_path(node, bandwidth):
    env.db.BANDWIDTH_ORIG = bandwidth
    path = [node]
    num_ue = len(node.user_list)
    rate = node.user_list[0].datarate

    drone_env = env.DroneDQNEnv()
    state = drone_env.reset(node)
    input_dim = state.shape[0]
    drone_agent = agent.Agent(
        input_dim=input_dim,
        output_dim=7,
        lr=1e-4,
        gamma=0.99,
        tau=1e-3,
        epsilon=0.03,
        epsilon_decay=1,
    )
    drone_agent.load_network(
        f"/home/hslyu/dbspf/tp_dqn/checkpoints/bw-{env.db.BANDWIDTH_ORIG}/ue-{num_ue}_rate-{rate}.pt"
    )

    state = drone_env.reset(node)
    while True:
        action = drone_agent.act(state)
        next_state, _, done, _ = drone_env.step(action)
        state = next_state
        path.append(drone_env.current_node)
        if done:
            break

    return path
