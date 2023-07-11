import torch as T
import torch.nn.functional as F
from start_reinforcement_learning.maddpg_algorithm.agent import Agent
import numpy as np
import torch

torch.autograd.set_detect_anomaly(True)

class MADDPG:
    def __init__(self, actor_dims, critic_dims, n_agents, n_actions, 
                 scenario='robot',  alpha=0.01, beta=0.01, fc1=512, 
                 fc2=512, gamma=0.99, tau=0.01, chkpt_dir='tmp/maddpg/', node_logger = None):
        self.agents = []
        self.n_agents = n_agents
        self.n_actions = n_actions
        self.logger = node_logger
        chkpt_dir += scenario 
        for agent_idx in range(self.n_agents):
            self.agents.append(Agent(actor_dims[agent_idx], critic_dims,  
                            n_actions, n_agents, agent_idx, alpha=alpha, beta=beta,
                            chkpt_dir=chkpt_dir))

    def save_checkpoint(self):
        print('... saving checkpoint ...')
        for agent in self.agents:
            agent.save_models()

    def load_checkpoint(self):
        print('... loading checkpoint ...')
        for agent in self.agents:
            agent.load_models()
    
    # Outputs two discrete actions from two continous action outputs from nn
    def discretize(self, continuous_actions):
        if continuous_actions[0] < 0.33:
            linear_velocity_action = 0
        elif continuous_actions[0] < 0.66:
            linear_velocity_action = 1
        else:
            linear_velocity_action = 2

        if continuous_actions[1] < 0.33:
            angular_velocity_action = 0
        elif continuous_actions[1] < 0.66:
            angular_velocity_action = 1
        else:
            angular_velocity_action = 2

        discrete_actions = np.array([linear_velocity_action, angular_velocity_action])
        return discrete_actions
                
    # returns dict of each agents chosen action for linear and velocity
    def choose_action(self, raw_obs):        
        actions = {}
        for agent_id, agent in zip(raw_obs, self.agents):
            continuous_actions = agent.choose_action(raw_obs[agent_id])
            #print(continuous_actions)
            discrete_actions = self.discretize(continuous_actions)            
            actions[agent_id] = discrete_actions
            
            
        return actions

    # Adjusts actor and critic wieghts
    def learn(self, memory):
        # If memory is not the size of a batch size (1024) then return
        if not memory.ready():
            return
        
        self.logger.get_logger().info("learning")
        # Samples algorithms central memory
        actor_states, states, actions, rewards, \
        actor_new_states, states_, dones = memory.sample_buffer()

        # Makes sure each tensor is working on the same device, should be gpu (cuda:0)
        device = self.agents[0].actor.device
        
        # converts sampled memory list of arrays to Tensors
        states = T.tensor(np.array(states), dtype=T.float).to(device)
        rewards = T.tensor(np.array(rewards), dtype=T.float).to(device)
        actions = T.tensor(np.array(actions), dtype=T.float).to(device)
        states_ = T.tensor(np.array(states_), dtype=T.float).to(device)
        dones = T.tensor(np.array(dones)).to(device)

        all_agents_new_actions = []
        all_agents_new_mu_actions = []
        old_agents_actions = []

        # For each agents individual memory
        for agent_idx, agent in enumerate(self.agents):
            # Create a tensor from the 'new state' The 'new state' results from 'action' chosen in 'state' all sampled from memory buffer
            new_states = T.tensor(actor_new_states[agent_idx], 
                                 dtype=T.float).to(device)
            # Output the action probabilities given the 'new state'
            new_pi = agent.target_actor.forward(new_states).to(device)
            # Store actions values
            all_agents_new_actions.append(new_pi)
            # Create a tensor from the 'state'
            mu_states = T.tensor(actor_states[agent_idx], 
                                 dtype=T.float).to(device)
            # Output the action probabilities given the 'state'
            pi = agent.actor.forward(mu_states).to(device)
            # Store actions values
            all_agents_new_mu_actions.append(pi)
            old_agents_actions.append(actions[agent_idx])

        # Create a tensor and store the concatonated action probabilities from 'next state' from each agent
        new_actions = T.cat([a for a in all_agents_new_actions], dim=1)
        # Same as above but for 'state'
        mu = T.cat([a for a in all_agents_new_mu_actions], dim=1)
        # Create a tensor and store the actions of every agent took from the 'previous state'
        old_actions = T.cat([a for a in old_agents_actions],dim=1)

        for agent_idx, agent in enumerate(self.agents):
            # Get 'new state' action value from every robot. .squeeze() removes every dimension of 1 from tensor
            critic_value_ = agent.target_critic.forward(states_,
                                                        new_actions).squeeze()
            # Get 'new state' action value from every robot. .squeeze() removes every dimension of 1 from tensor
            critic_value = agent.critic.forward(states, old_actions).squeeze()
            # Updated the done flags on the centralized critic
            critic_value_[dones[:, 0]] = 0.0
            # Calculate target params
            target = rewards[:, agent_idx] + agent.gamma*critic_value_.detach()
            # Calculate loss from target and critic value
            critic_loss = F.mse_loss(critic_value, target)

            actor_loss = -agent.critic.forward(states, mu).flatten()
            actor_loss = T.mean(actor_loss)

            agent.actor.optimizer.zero_grad()
            actor_loss.backward(retain_graph=True)

            agent.critic.optimizer.zero_grad()
            critic_loss.backward(retain_graph=True)

        for agent_idx, agent in enumerate(self.agents):
            agent.actor.optimizer.step()
            agent.critic.optimizer.step()
            agent.update_network_parameters()
