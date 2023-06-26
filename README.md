# testcm30082ros

## Installation

Install Ubunty 22.04 OS

Install [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

Permanantly source humble environment
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
Download workspace
```
git clone https://github.com/i1Cps/testcm30082ros.git

```

Install package dependencies
```
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
pip install setuptools==58.2.0
colcon build --symlink-install
```

Update [graphics drivers](https://beebom.com/how-install-drivers-ubuntu/)
```
sudo ubuntu-drivers autoinstall
sudo reboot
```
