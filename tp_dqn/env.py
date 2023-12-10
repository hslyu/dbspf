import random
import sys
from os import path

import torch

sys.path.append(path.dirname(path.abspath(path.dirname(__file__))))
import drone_basestation as db  # noqa


class DroneDQNEnv:
    def __init__(
        self,
        vehicle_velocity=15,
        time_step=3,
        grid_size=40,
        map_width=600,
        min_altitude=50,
        max_altitude=200,
        max_timeslot=20,
        num_ue=20,
        initial_data=10,
        time_window_size=[4, 8],
        time_period_size=[20, 20],
        datarate_window=[1, 1],
    ):
        self.current_node = db.TrajectoryNode([0, 0, 0])
        self.vehicle_velocity = vehicle_velocity
        self.time_step = time_step
        self.grid_size = grid_size
        self.map_width = map_width
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.max_timeslot = max_timeslot
        self.num_ue = num_ue
        self.initial_data = initial_data
        self.time_window_size = time_window_size
        self.time_period_size = time_period_size
        self.datarate_window = datarate_window
        return

    def step(self, action):
        position = self._compute_position(action)
        self.current_node = db.TrajectoryNode(position, parent=self.current_node)
        terminated = self.current_node.current_time >= self.max_timeslot
        next_state = self.compute_state()

        return next_state, self.current_node.reward, terminated, False

    def reset(self, node=None) -> torch.Tensor:
        if node is not None:
            self.current_node = node
            return self.compute_state()

        position = [
            random.randint(0, self.map_width) // 10 * 10,
            random.randint(0, self.map_width) // 10 * 10,
            random.randint(self.min_altitude, self.max_altitude) // 10 * 10,
        ]

        user_list = []
        for i in range(self.num_ue):
            tw_size = random.randint(self.time_window_size[0], self.time_window_size[1])
            time_period = random.randint(
                self.time_period_size[0], self.time_period_size[1]
            )
            datarate = random.randint(self.datarate_window[0], self.datarate_window[1])
            user = db.User(
                i,  # id
                [
                    random.randint(0, self.map_width) // 10 * 10,
                    random.randint(0, self.map_width) // 10 * 10,
                ],  # position
                random.randint(0, self.max_timeslot - tw_size),  # time_start
                tw_size,
                time_period,  # time window
                datarate,
                self.initial_data,
                max_data=99999,
            )  # data
            user_list.append(user)

        node = db.TrajectoryNode(position)
        node.user_list = user_list
        self.current_node = node

        return self.compute_state()

    def compute_state(self) -> torch.Tensor:
        """
        Vectorize the node information as a Tensor
        Args:
            node (TrajectoryNode): TrajectoryNode
        return:
            state (Tensor): state
        """
        state = []
        state += self._compute_next_position_reward()
        state += [self.current_node.current_time / self.max_timeslot]
        state += [
            self.current_node.position[0] / self.map_width,
            self.current_node.position[1] / self.map_width,
            self.current_node.position[2] / self.max_altitude,
        ]
        for user in self.current_node.user_list:
            finished = 1
            if user.time_start <= self.current_node.current_time <= user.time_end:
                finished = 0
            user_state = [
                (user.position[0] - self.current_node.position[0]) / self.map_width,
                (user.position[1] - self.current_node.position[1]) / self.map_width,
                finished,
                user.time_start / self.max_timeslot,
                user.time_end / self.max_timeslot,
                user.pathloss / 140,
                user.datarate / 10,
                user.received_data / 20,
            ]
            state += user_state

        state = torch.Tensor(state)
        return state

    def _compute_next_position_reward(self):
        reward_list = []
        for i in range(7):
            position = self._compute_position(i)
            if position == self.current_node.position and i != 0:
                reward_list.append(-1)
                continue
            node = db.TrajectoryNode(position, parent=self.current_node)
            reward = node.reward
            reward_list.append(reward)

        return reward_list

    def _isAvailable(self, position):
        isAvailable = False
        # If the position is in the map, return true.
        isAvailable = (
            0 <= position[0] <= self.map_width
            and 0 <= position[1] <= self.map_width
            and self.min_altitude <= position[2] <= self.max_altitude
        )
        ############################################################
        # If there's any forbidden place in the map, write code here
        # isAvailable = isAvailable and (code here)
        ############################################################
        # otherwise return false.
        return isAvailable

    def _compute_position(self, action: int):
        """
        (Warning) This funciton is optimized for the 6 movement cases.
                  If there're more than 6 positions where the drone can move,
                  it doesn't work correctly.
        args:
            action (int): action 0-6
        return:
            position (list): next node position
        """
        position = self.current_node.position
        next_position = position.copy()
        if action == 0:
            return position
        elif action == 1:
            next_position[0] += self.grid_size
        elif action == 2:
            next_position[0] -= self.grid_size
        elif action == 3:
            next_position[1] += self.grid_size
        elif action == 4:
            next_position[1] -= self.grid_size
        elif action == 5:
            next_position[2] += self.grid_size
        elif action == 6:
            next_position[2] -= self.grid_size
        return next_position if self._isAvailable(next_position) else position


def main():
    env = DroneDQNEnv(
        vehicle_velocity=15,
        time_step=3,
        grid_size=45,
        map_width=600,
        min_altitude=50,
        max_altitude=200,
        max_timeslot=20,
        num_ue=40,
        initial_data=1,
        time_window_size=[4, 8],
        time_period_size=[20, 20],
        datarate_window=[1, 1],
    )
    state = env.reset()
    print(state.shape[0])


if __name__ == "__main__":
    main()
