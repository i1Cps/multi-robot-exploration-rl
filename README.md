# Robotic Exploration using Machine Learning
## How to use (After installation)

### To start up the gazebo environment with the robots open a fresh terminal and run:
```
cd robotic_exploration_ml
source install/setup.bash
ros2 launch start_rl_environment main.launch.py
```

### To begin training the robots open a new fresh terminal and run:
```
cd robotic_exploration
source install/setup.bash
ros2 launch start_reinforcement_learning start_learning.launch.py
```

## Installation

### Install Ubunty 22.04 OS

### Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

Permanantly source humble environment
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
### Download workspace
```
git clone https://github.com/i1Cps/robotic_exploration_ml.git
cd robotic_exploration_ml

```

### Install package dependencies
```
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
pip install setuptools==58.2.0
colcon build --symlink-install
```

### Update [graphics drivers](https://beebom.com/how-install-drivers-ubuntu/)
```
sudo ubuntu-drivers autoinstall
sudo reboot
```
### WARNING - Sudo reboot will restart your system
