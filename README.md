<!-- ⚠️ This README has been generated from the file(s) "blueprint.md" ⚠️--><h1 align="center">multi-robot-exploration_rl</h1>
<p align="center">
  <img src="images/maze.png" alt="Logo" width="550" height="auto" />
</p>


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#table-of-contents)

## ➤ Table of Contents

* [➤ ::pencil:: About The Project](#-pencil-about-the-project)
* [➤ :rocket: Dependencies](#-rocket-dependencies)
* [➤ ::pencil:: About The Project](#-pencil-about-the-project-1)
* [➤ :coffee: Buy me a coffee](#-coffee-buy-me-a-coffee)
* [➤ :scroll: Credits](#-scroll-credits)
* [➤ License](#-license)


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#pencil-about-the-project)

## ➤ ::pencil:: About The Project

This project employs a centralized critic network and decentralized actor networks to facilitate cooperative behaviour among multiple robots. The simulation environment is implemented in Gazebo, utilizing ROS2's capabilities, to demonstrate and test this multi-agent deep reinforcement learning (MADDPG) approach.

The project contains two ROS2 packages:

`start_rl_environment`

and

`start_reinforcement_learning`


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#rocket-dependencies)

## ➤ :rocket: Dependencies

  

The MADDPG incorporates Pytorch while the environment uses Gazebo and ROS 2

![PyTorch Badge](https://img.shields.io/badge/PyTorch-EE4C2C?logo=pytorch&logoColor=fff&style=for-the-badge) ![ROS Badge](https://img.shields.io/badge/ROS-22314E?logo=ros&logoColor=fff&style=for-the-badge)
![Python Badge](https://img.shields.io/badge/Python-3776AB?logo=python&logoColor=fff&style=for-the-badge) ![suttin](https://custom-icon-badges.demolab.com/badge/-GazeboSim-FFBF00?style=for-the-badge&logo=package&logoColor=black)


[![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/cloudy.png)](#pencil-about-the-project)

## ➤ ::pencil:: About The Project

1.  **`start_rl_environment`**: This package simulates the robots' training environment. It sets the stage for their learning journey by providing a realistic virtual environment in which they can develop their skills.
    
2.  **`start_reinforcement_learning`**: This package is dedicated to launching and executing the Multi-Agent Deep Deterministic Policy Gradient (MADDPG) algorithm. Leveraging reinforcement learning techniques, it guides the robots through their training process to solve autonomous exploration in a multi-robot context.

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
