<!-- ⚠️ This README has been generated from the file(s) "blueprint.md" ⚠️--><h1 align="center">multi-robot-exploration_rl</h1>
<p align="center">
  <img src="images/maze.png" alt="Logo" width="550" height="auto" />
</p>


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#table-of-contents)

## ➤ Table of Contents

* [➤ ::pencil:: About The Project](#-pencil-about-the-project)
* [➤ :factory: `start_rl_environment` ROS 2 Package](#-factory-start_rl_environment-ros-2-package)
	* [_Package Overview:_](#_package-overview_)
	* [_Key Features and Functionalities:_](#_key-features-and-functionalities_)
	* [_Usage:_](#_usage_)
	* [_Purpose:_](#_purpose_)
* [➤ :robot: `start_reinforcement_learning` ROS 2 Package](#-robot-start_reinforcement_learning-ros-2-package)
	* [_Package Overview:_](#_package-overview_-1)
	* [_Key Features and Functionalities:_](#_key-features-and-functionalities_-1)
	* [_Usage:_](#_usage_-1)
	* [_Purpose:_](#_purpose_-1)
* [➤ :floppy_disk: Key Project File Descriptions](#-floppy_disk-key-project-file-descriptions)
	* [`start_rl_environment`](#start_rl_environment)
		* [Robot Model Files](#robot-model-files)
		* [Launch Files](#launch-files)
		* [World Files](#world-files)
	* [`start_reinforcement_learning`](#start_reinforcement_learning)
		* [MADDPG](#maddpg)
		* [Environment Logic](#environment-logic)
		* [Launch Files](#launch-files-1)
* [➤ :rocket: Dependencies](#-rocket-dependencies)
* [➤ :hammer: Basic Installation](#-hammer-basic-installation)
* [➤ :coffee: Buy me a coffee](#-coffee-buy-me-a-coffee)
* [➤ :scroll: Credits](#-scroll-credits)
* [➤ License](#-license)


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#pencil-about-the-project)

## ➤ ::pencil:: About The Project

This project employs a centralized critic network and decentralized actor networks to facilitate cooperative behaviour among multiple robots. The simulation environment is implemented in Gazebo, utilizing ROS2's capabilities. The robots are then trained using a Multi-Agent Deep Reinforcement Learning approach.

The project contains two ROS2 packages:

[**`start_rl_environment`**](#:factory:-`start_rl_environment`-ROS-2-Package)

and

[**`start_reinforcement_learning`**](#:robot:-`start_reinforcement_learning`-ROS-2-Package)


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#factory-start_rl_environment-ros-2-package)

## ➤ :factory: `start_rl_environment` ROS 2 Package

### _Package Overview:_

The `start_rl_environment` package seamlessly integrates [ROS 2 Humble](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html) and [Gazebo](https://gazebosim.org/home) services, serving as a critical component within the project. Its primary responsibility is establishing and overseeing the simulated training environment for autonomous robotic agents. This pivotal role involves configuring an environment in which robots can cultivate their navigation and exploration skills through the application of Reinforcement Learning (RL).

### _Key Features and Functionalities:_

1.  **Robot Modelling**: Within `start_rl_environment`, you can locate the robot model in the `description` folder, represented by several `.xacro` files adhering to the Unified Robot Description Format (URDF). These files comprehensively capture the physical attributes, kinematics, and sensor configurations of the robots utilised in the simulation.
    
2.  **Map Simulation**: Within `start_rl_environment`, the map models can be found in the worlds folder. Each map model is constructed using a .world file that integrates individual .sdf files, simulating the physical layout of the environment. This comprehensive representation can include terrain, obstacles, landmarks, and other elements crucial for creating a realistic training scenario.
    
3. **Sensor Integration**: Sensor models and configurations, such as cameras, LiDARs, or other relevant sensors, can be included and configured within the package to mimic real-world sensor data collection.
    
4. **Launch System**: To simultaneously facilitate multiple robots' training, the package employs a sophisticated launch system implemented in .py files. This launch system ensures that numerous robot instances can be spawned and controlled within the simulated environment.
    
### _Usage:_

-   To utilise `start_rl_environment`, please follow these steps after completing the basic installation:

1. Navigate to the project directory:
	```
	cd robotic_exploration_ml
	```
2. Source the ROS 2 environment:
	```
	source /opt/ros/humble/setup.bash
	```
3. Source the project environment:
	```
	source install/setup.bash
	```
4. Launch the environment with specific map and robot configurations using the following command:
	```
	ros2 launch start_rl_environment main.launch.py --map_number <map_number> --robot_number <robot_number>
	```

Replace the optional argument `<map_number>` with either `1` or `2` to specify the map for the simulation (default: 1).
Replace the optional argument `<robot_number>` with an integer between `1` and `7` to specify the number of robots in the simulation (default: 3).

**WARNING**:   MAKE SURE THE ARGUMENTS BETWEEN `start_rl_environemnt` and `start_reinforcement_learning` ARE THE SAME 

### _Purpose:_

The primary objective of `start_rl_environment` is to create a realistic and adaptable training environment for the robots. Map 1 represents a simple corridor the robots can search, while map 2 has an L-shaped room to further illustrate the benefit of multi-robot exploration over singular exploration.

This package is a fundamental building block in achieving the overarching goal of multi-robot autonomous exploration using deep reinforcement learning.                           


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#robot-start_reinforcement_learning-ros-2-package)

## ➤ :robot: `start_reinforcement_learning` ROS 2 Package

### _Package Overview:_

The `start_reinforcement_learning` package applies the Multi-Agent Deep Deterministic Policy Gradient (MADDPG) algorithm to each robot in a decentralised-centralised fashion. It leverages PyTorch and CUDA to empower our autonomous robots with cooperative learning capabilities, facilitating efficient exploration.

### _Key Features and Functionalities:_

1.  **MADDPG Algorithm**: The [MADDPG](https://dl.acm.org/doi/10.5555/3295222.3295385) algorithm operates as an Actor-Critic algorithm, combining decentralisation during execution with centralised training. This unique approach offers significant advantages for cooperative learning among autonomous robots.

	We allow each robot to use additional information about other robots during the training phase while ensuring that the extra information is not utilised during execution. This centralisation of training data enhances learning efficiency and cooperative behaviours among the robots, enabling them to collaborate effectively in complex environments.
    
2.  **Reward Function**: The package also handles reward computation. (Please note the current reward function in the code is a simple example for obvious reasons my reward function is not open source yet.

3. **Episode Logic**:  The "Episode Logic" within the `start_reinforcement_learning` package includes collision handling, where robots receive negative rewards for colliding with obstacles, promoting obstacle avoidance. Goal achievement is encouraged as robots aim to reach specific goals, resulting in rewards upon success. To ensure efficiency in training, episodes are truncated if the maximum duration (MAX_STEPS) is reached without goal attainment. This episode logic structure enhances the training process, enabling emrging behaviours and autonomous adaptation in complex environments.
    
5.  **Action Space Definition**: `start_reinforcement_learning` defines and manages the action space for each robot, enabling intelligent decision-making. The Action space can be optimized for both continuous and discrete action but it should be noted due to the nature of MADDPG, a continuous action space is more natural (Once again please note the current action space in the code is not final for my research and for obvious reasons cannot be revealed yet)
    
5. **Integration of Sensor Data and Simulation Communication**: The `start_reinforcement_learning` package seamlessly connects sensor data integration and simulation communication. It employs ROS 2's topic-based communication to establish a vital link between the Gazebo simulation environment and the package's environment logic. This mechanism efficiently exchanges sensor data between nodes, mainly LiDAR, command velocity and odometry data. This integration facilitates realistic training and empowers robots to enhance their perception and decision-making abilities.
    

### _Usage:_

-   To utilise `start_reinforcement_learning`, please follow these steps after completing the basic installation:

1. Navigate to the project directory:
	```
	cd robotic_exploration_ml
	```
2. Source the ROS 2 environment script:
	```
	source /opt/ros/humble/setup.bash
	```
3. Source the project environment:
	```
	source install/setup.bash
	```
4. Launch the MADDPG training process using the following command:
	```
	ros2 launch start_reinforcement_learning start_learning.launch.py map_number:=<map_number> robot_number:=<robot_number>
	```
	Replace the optional argument `<map_number>` with either `1` or `2` to specify the map for the simulation (default: 1).

	Replace the optional argument `<robot_number>` with an integer between `1` and `7` to specify the number of robots in the simulation (default: 3).

**WARNING**:   MAKE SURE THE ARGUMENTS BETWEEN `start_rl_environemnt` and `start_reinforcement_learning` ARE THE SAME.

### _Purpose:_

The primary purpose of the `start_reinforcement_learning` package is to enable multi-robot autonomous exploration using state-of-the-art MADDPG reinforcement learning techniques. By facilitating cooperative learning and enhancing exploration skills, this package empowers robots to efficiently navigate and explore complex environments. It serves as a vital component in achieving our project's goal of autonomous exploration in real-world scenarios.



[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#floppy_disk-key-project-file-descriptions)

## ➤ :floppy_disk: Key Project File Descriptions

### `start_rl_environment`
#### Robot Model Files
* `robot_core.xacro`: The main file that describes the visual, collision and inertia properties of the robots
* `lidar.xacro`: Defines the property of LiDARs on each robot
#### Launch Files
* `main.launch.py`: This is the main launch file; it makes two further calls to the `start_world.launch.py` file and the `start_robots.launch.py` file. It takes two optional arguments: map_number for the map you want to use and robot_number for the number of robots you want to train.

* `start_world.launch.py`: This launch file handles the simulation of the Gazebo environment by calling the `gazebo.launch.py` file with our custom map as an argument (`gazebo.launch.py` was developed by [gazebosim](https://gazebosim.org/home)).
* `start_robots.launch.py`: This launch file handles the simulation of multiple robots by creating an array of properties for each robot the user wants to train. This includes name, namespace, position and rotation and URDF description of the robot. Then all the properties are fed via [substitutions](https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Using-Substitutions.html#) to a launch_description so that the properties of each robot can individually be spawned by the `spawn_robots.launch.py` file.

* `spawn_robots.launch.py`: This file is responsible for spawning any robot passed to it; it first accesses the robot properties by unpacking the launch substitutions from `start_robots.launch.py`. It then feeds those properties to the Gazebo 'spawn_entity' Node. Which will spawn the robot on the first gazebo server it finds running. It then calls the robot state publisher node, responsible for publishing the robot's state information, typically related to its pose (position and orientation) in a specific reference frame.

#### World Files
* `map1.world`: The file responsible for building the map 1 world



### `start_reinforcement_learning`
#### MADDPG
* `maddpg_main.py`: This is the main file that runs the individual components of the maddpg algorithm; it contains important constants such as the number of episodes, Amount of memory allocated to the buffer (It's currently set to a '1000000', **PLEASE** adjust to something lower if you are not using 32GB RAM), the dimension space of the actor and critic networks various other hyperparamers.

* `agent.py`: The `Agent` class encapsulates the behaviour of individual agents within the MADDPG algorithm. It includes actor and critic neural networks for decision-making and learning, respectively. Utilising target networks to enhance training stability, the agent's `choose_action` method translates observations into action probabilities based on the actor's weights, optionally adding noise for exploration. Additionally, the class offers functionalities for saving and loading model checkpoints, making it an integral component of cooperative multi-agent reinforcement learning.

* `buffer.py`: The `MultiAgentReplayBuffer` class manages experiences in multi-agent reinforcement learning, organising data for training. It maintains separate memory stores for each agent, efficiently handling transitions, including raw observations, states, actions, rewards, next states, and terminal flags. The `sample_buffer` method randomly selects samples for training, while the `ready` function checks if enough samples are available for learning to start.

* `networks.py`:
	* **CriticNetwork:** The `CriticNetwork` class is a fundamental component of multi-agent reinforcement learning. It plays a crucial role in evaluating state-action pairs and computing Q-values, which is essential for learning optimal policies. This neural network takes various parameters, including input dimensions, layer dimensions, the number of agents, actions, a name, and a checkpoint directory. The `forward` method processes state-action pairs to produce Q-values. The Adam optimiser manages the network's optimisation, making it adaptable to different environments. Furthermore, it offers the ability to save and load model checkpoints, ensuring continuity in training and evaluation.

	* **ActorNetwork:** The `ActorNetwork` class complements the `CriticNetwork` in multi-agent reinforcement learning by generating agent actions based on observed states. Like the `CriticNetwork`, it requires parameters such as input dimensions, layer dimensions, the number of actions, a name, and a checkpoint directory. In the `forward` method, this neural network computes action probabilities given a state. It uses the softmax activation function, making it suitable for probabilistic action selection. Like its counterpart, the `ActorNetwork` is optimised with the Adam optimiser and supports the saving and loading of model checkpoints. This functionality ensures that learned policies can be preserved and used in various scenarios.

* `maddpg.py`: The `MADDPG` class is the core orchestrator in multi-agent deep deterministic policy gradients (MADDPG) reinforcement learning. It facilitates the training and coordination of multiple agents in a collaborative or competitive environment. This class initialises a group of agents, each represented by an instance of the `Agent` class, with user-defined parameters such as actor and critic neural network architectures, learning rates, exploration noise, and more. It provides methods for selecting actions, learning from experiences stored in a replay buffer, saving and loading model checkpoints, and discretising continuous actions if required. The `learn` method orchestrates the learning process, where agents update their actor and critic networks to improve their policies based on shared experiences. Overall, the `MADDPG` class encapsulates the logic necessary for training and deploying multiple agents in complex environments, making it a fundamental component of multi-agent reinforcement learning systems.


#### Environment Logic
* `logic.py`: The `Env` class orchestrates the simulation environment for multiple robots, interfacing with ROS for communication. It manages robot movements, sensor data collection, collision detection, and goal-reaching. The class handles episodic aspects, tracking step counts and episode terminations. It maintains robot states and velocities, discretizes actions, and calculates rewards based on robot behaviours. In essence, it acts as the intermediary between multi-agent reinforcement learning algorithms and the robotic simulations, serving a pivotal role in agent training and evaluation.

* `restart_environment.py`: The `RestartEnvironment` class handles the logic when a robot terminates or reaches its goal. One feature is that when an episode ends, each robot is teleported to a safe location while the training area is cleaned up, then teleported to its original starting position to begin learning again.

#### Launch Files
* `start_learning.launch.py`: This file launches the `maddpg_main.py` file. t takes two optional arguments: map_number for the map you want to use and robot_number for the amount of robots you want to train.






[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#rocket-dependencies)

## ➤ :rocket: Dependencies

The MADDPG algorithm incorporates Pytorch while the environment uses Gazebo and ROS 2

![PyTorch Badge](https://img.shields.io/badge/PyTorch-EE4C2C?logo=pytorch&logoColor=fff&style=for-the-badge) ![ROS Badge](https://img.shields.io/badge/ROS-22314E?logo=ros&logoColor=fff&style=for-the-badge)
![Python Badge](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=fff&style=for-the-badge) ![suttin](https://custom-icon-badges.demolab.com/badge/-GazeboSim-FFBF00?style=for-the-badge&logo=package&logoColor=black)


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#hammer-basic-installation)

## ➤ :hammer: Basic Installation

 **Install Ubuntu 22.04 OS**

**Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)**

**Download workspace**
```
git clone https://github.com/i1Cps/robotic_exploration_ml.git
cd robotic_exploration_ml
```

**Install package dependencies**
```
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
pip install setuptools==58.2.0
colcon build --symlink-install
```

**Update [graphics drivers](https://beebom.com/how-install-drivers-ubuntu/)**
```
sudo ubuntu-drivers autoinstall
sudo reboot
```
**WARNING - Sudo reboot will restart your system**

[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#coffee-buy-me-a-coffee)

## ➤ :coffee: Buy me a coffee
Whether you use this project, have learned something from it, or just like it, please consider supporting it by buying me a coffee, so I can dedicate more time on open-source projects like this (҂⌣̀_⌣́)

<a href="https://www.buymeacoffee.com/i1Cps" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/v2/default-violet.png" alt="Buy Me A Coffee" style="height: 60px !important;width: 217px !important;" ></a>



[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#scroll-credits)

## ➤ :scroll: Credits

Theo Moore-Calters 


[![GitHub Badge](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/i1Cps) [![LinkedIn Badge](https://img.shields.io/badge/LinkedIn-0077B5?style=for-the-badge&logo=linkedin&logoColor=white)](www.linkedin.com/in/theo-moore-calters)


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#license)

## ➤ License
	
Licensed under [MIT](https://opensource.org/licenses/MIT).
