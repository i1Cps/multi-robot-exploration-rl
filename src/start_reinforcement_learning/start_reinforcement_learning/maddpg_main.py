import os
import rclpy
from rclpy.node import Node
import numpy as np

from start_reinforcement_learning.env_logic.logic import Env
from start_reinforcement_learning.maddpg_algorithm.maddpg import MADDPG
from start_reinforcement_learning.maddpg_algorithm.buffer import MultiAgentReplayBuffer
import torch as T
import gc
from ament_index_python.packages import get_package_share_directory

# Convert list of arrays to one flat array of observations
def obs_list_to_state_vector(observation):
    state = np.array([])    
    for obs in observation:
        state = np.concatenate([state, obs])
    return state

# Main function that runs the MADDPG algorithnm
class MADDPGNode(Node):
    def __init__(self, map_number, robot_number):
        super().__init__('maddpg_node')

        # Access the parameters passed from the launch file
        map_number = self.declare_parameter('map_number', 1).get_parameter_value().integer_value
        robot_number = self.declare_parameter('robot_number', 3).get_parameter_value().integer_value

        self.get_logger().info(f"Map nuumber: {map_number}")
        self.get_logger().info(f"Robot number: {robot_number}")
        #T.cuda.empty_cache()
        #gc.collect()

        # Set environment with action size
        env = Env()
        self.get_logger().info(f"Map nuumber: {map_number}")
        n_agents = env.number_of_robots
        
        actor_dims = env.observation_space()
        critic_dims = sum(actor_dims)

        # Action space is discrete, one of 4 actions,  look in env
        n_actions = env.action_space()

        chkpt_dir_var = os.path.join(get_package_share_directory('start_reinforcement_learning'),
                                    'start_reinforcement_learning','deep_learning_weights','maddpg')
        
        # Initialize main algorithm
        maddpg_agents = MADDPG(actor_dims, critic_dims, n_agents, n_actions, 
                               fc1=512, fc2=512, tau=0.00025,
                               alpha=1e-4, beta=1e-3, scenario='robot',
                               chkpt_dir=chkpt_dir_var)

        # Initialize memory
        memory = MultiAgentReplayBuffer(1000000, critic_dims, actor_dims, 
                            n_actions, n_agents, batch_size=10240)

        PRINT_INTERVAL = 10
        N_GAMES = 5000
        total_steps = 0
        score_history = []
        evaluate = False
        best_score = 0

        # Test network, remember decentralised centralised network,  training include critic + actor, testing only includes actor
        if evaluate:
            maddpg_agents.load_checkpoint()

        # Currently 500 episodes
        for i in range(N_GAMES):
            # reset to get initial observation
            obs = env.reset()
            # Convert dict -> list of arrays to go into 'obs_list_to_state_vector' function
            list_obs = list(obs.values())
            score = 0
            done = [False]*n_agents
            terminal = [False] * n_agents
            episode_step = 0
            run_ep = True
            # Truncated means episode has reached max number of steps, done means collided or reached goal
            while not any(terminal):
                # Get the actions that the algorithm thinks are best in given observation
                actions = maddpg_agents.choose_action(obs)
                # use step function to get next state and reward info as well as if the episode is 'done'
                obs_, reward, done, truncated, info = env.step(actions)
                
                # Convert dict -> list of arrays to go into 'obs_list_to_state_vector' function
                list_done = list(done.values())
                list_reward = list(reward.values())
                list_actions = list(actions.values())
                list_obs_ = list(obs_.values())
                list_trunc = list(truncated.values())

                # Convert list of arrays to one flat array of observations
                state = obs_list_to_state_vector(list_obs)
                state_ = obs_list_to_state_vector(list_obs_)
                
                terminal = [d or t for d, t in zip(list_done, list_trunc)]

                # Store raw observation as well as list of each agent's observation, reward, and done value together
                memory.store_transition(list_obs, state, list_actions, list_reward, list_obs_, state_, list_done)

                if total_steps % 500 == 0 and not evaluate:
                    maddpg_agents.learn(memory)

                # Set new obs to current obs
                obs = obs_
                score += sum(list_reward)
                total_steps += 1
                episode_step += 1

            score_history.append(score)
            avg_score = np.mean(score_history[-100:])
            if not evaluate:
                if avg_score > best_score:
                    maddpg_agents.save_checkpoint()
                    best_score = avg_score
            if i % PRINT_INTERVAL == 0 and i > 0:
                self.get_logger().info('Episode: {}, Average score: {:.1f}'.format(i, avg_score))


def main(args=None):
    rclpy.init(args=args)
    
    map_number = int(os.getenv('map_number', '1'))
    robot_number = int(os.getenv('robot_number', '3'))
    node = MADDPGNode(map_number, robot_number)
    #rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()