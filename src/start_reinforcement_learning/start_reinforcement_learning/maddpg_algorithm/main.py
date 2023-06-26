import numpy as np
from maddpg import MADDPG
from buffer import MultiAgentReplayBuffer
#from make_env import make_env
from pettingzoo.mpe import simple_adversary_v2
import torch as T
import gc

# Convert list of arrays to one flat array of observations
def obs_list_to_state_vector(observation):
    state = np.array([])    
    for obs in observation:
        state = np.concatenate([state, obs])
    return state

if __name__ == '__main__':
   
    
    T.cuda.empty_cache()
    gc.collect()

    parallel_env = simple_adversary_v2.parallel_env()
    scenario = 'simple_adversary'
    
    # I only call reset outside the main loop to access env properties below
    initial_temp = parallel_env.reset()
    n_agents = parallel_env.num_agents
    agents = parallel_env.agents
    
    actor_dims = []
    
    for agent in parallel_env.agents:
        actor_dims.append(parallel_env.observation_space(agent).shape[0])
    # simple_adversary = 10 + 10 + 8 = 28
    critic_dims = sum(actor_dims)

    # Action space is a list of arrays, assume each agent has same action space
    # Since we assume each agent has the same action space we just take the action space of the first agent
    n_actions = parallel_env.action_space(agents[0]).n
    
    # Initialize main algorithm
    maddpg_agents = MADDPG(actor_dims, critic_dims, n_agents, n_actions, 
                           fc1=64, fc2=64,  
                           alpha=0.01, beta=0.01, scenario=scenario,
                           chkpt_dir='tmp/maddpg/')

    # Initialize memory
    memory = MultiAgentReplayBuffer(100000, critic_dims, actor_dims, 
                        n_actions, n_agents, batch_size=1024)


    PRINT_INTERVAL = 500
    N_GAMES = 50000
    MAX_STEPS = 25
    total_steps = 0
    score_history = []
    evaluate = False
    best_score = 0

    if evaluate:
        maddpg_agents.load_checkpoint()
    # Currently 50000 episodes
    for i in range(N_GAMES):
        # reset to get initial observation
        obs = parallel_env.reset()
        # Convert dict -> list of arrays to go into 'obs_list_to_state_vector' function
        list_obs = list(obs.values())

        score = 0
        done = [False]*n_agents
        episode_step = 0
        # I need to add truncated to this while loop condition when code actually runs
        while not any(done):
            if evaluate:
                parallel_env.render()
                #time.sleep(0.1) # to slow down the action for the video
            
            # Get the actions that the algorithm thinks are best in given observation
            actions = maddpg_agents.choose_action(obs)
            # use step function to get next state and reward info as well as if the episode is 'done'
            obs_, reward, done, truncated, info = parallel_env.step(actions)
            # Convert dict -> list of arrays to go into 'obs_list_to_state_vector' function
            list_done = list(done.values())
            list_reward = list(reward.values())
            list_actions = list(actions.values())
            list_obs_ = list(obs_.values())
            
            # Convert list of arrays to one flat array of observations
            state = obs_list_to_state_vector(list_obs)
            state_ = obs_list_to_state_vector(list_obs_)

            # Every 25 steps episode is over
            if episode_step >= MAX_STEPS:
                done = [True]*n_agents

            # Store raw observation as well as list of each agents observation, reward, and done value together
            memory.store_transition(list_obs, state, list_actions, list_reward, list_obs_, state_, list_done)

            # Only call learn function every 100 steps, or every 4 episodes 
            if total_steps % 100 == 0 and not evaluate:
                maddpg_agents.learn(memory)

            # Set new obs to current obs
            obs = obs_
            score += sum(list_reward)
            total_steps += 1
            episode_step += 1
        # Keep track of score and stuff
        score_history.append(score)
        avg_score = np.mean(score_history[-100:])
        if not evaluate:
            if avg_score > best_score:
                maddpg_agents.save_checkpoint()
                best_score = avg_score
        if i % PRINT_INTERVAL == 0 and i > 0:
            print('episode', i, 'average score {:.1f}'.format(avg_score))