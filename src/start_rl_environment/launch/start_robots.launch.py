import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.substitutions import PythonExpression, LaunchConfiguration, PathJoinSubstitution

import xacro
import math
import random
import numpy as np
from simple_launch import SimpleLauncher

# Declare simple launcher and launch arguments in the main body
sl = SimpleLauncher()
sl.declare_arg('map_number', default_value=1)
sl.declare_arg('robot_number', default_value=3)
# Function which generates a list of x robots with inital poses
def gen_robot_list(number_of_robots, map_number):
    # Initial robot locations and rotations [x,y,yaw] for map 1
    map1_position1 = [-0.6,-10,(math.pi)/4]
    map1_position2 = [1.8,-10,(3*math.pi/4)]
    map1_position3 = [1.8,-1,(5*math.pi)/4]
    map1_position4 = [-0.6,-1,(-math.pi)/4]
    map1_position5 = [-0.94,-5.1,0]
    map1_position6 = [1.94,-5.1, -math.pi]
    map1_position7 = [0.47, -3.02, (-math.pi)/2]
    
    # Initial robot locations and rotations [x,y,yaw] for map 2
    map2_position1 = [9.2,-3,(3*math.pi)/4]
    map2_position2 = [9.2,-1.07,(-3*math.pi/4)]
    map2_position3 = [-0.55,-1.2,(-math.pi)/4]
    map2_position4 = [-0.8,-10.2,(math.pi)/4]
    map2_position5 = [-0.93,-5.1,0]
    map2_position6 = [1.93,-5.1, math.pi]
    map2_position7 = [1.53,-9.9,(3*math.pi)/4]
    
    # map 1 array of random locations for robots
    map1_random_robot_locations = [map1_position1,map1_position2,map1_position3,
                                   map1_position4,map1_position5,map1_position6, map1_position7]
    
    # map 2 array of random locations for robots
    map2_random_robot_locations = [map2_position1, map2_position2, map2_position3,
                                   map2_position4, map2_position5, map2_position6, map2_position7]
    
    # Random indicies map1
    map1_indicies = np.random.choice(len(map1_random_robot_locations), number_of_robots,False)

    # Random inicies map2
    map2_indicies = np.random.choice(len(map2_random_robot_locations), number_of_robots, False)
    
    # Selected robot locations for map 1
    map1_selected_robot_locations = np.zeros((number_of_robots,3))
    for i in range(number_of_robots):
        map1_selected_robot_locations[i] = map1_random_robot_locations[map1_indicies[i]]
        
    # Selected robot locations for map 2
    map2_selected_robot_locations = np.zeros((number_of_robots,3))
    for i in range(number_of_robots):
        map2_selected_robot_locations[i] = map2_random_robot_locations[map2_indicies[i]]
    
    #selected_robot_locations = map1_selected_robot_locations
    
    # This chooses the selected robot decision based on what type of map the user choose
    if map_number == 1:
        selected_robot_locations = map1_selected_robot_locations
    else:
        selected_robot_locations = map2_selected_robot_locations
    
    robots = []
    # Robot properties
    for i in range(number_of_robots):
        robot_name = "my_bot"+str(i)
        robot_name_space = "my_bot_ns" + str(i)
        robots.append({'name': robot_name, 'ns': robot_name_space, 'x_pose': selected_robot_locations[i][0],
                       'y_pose': selected_robot_locations[i][1], 'z_pose': 0.3,
                       'rotation': selected_robot_locations[i][2]})
    return robots 

def launch_setup():
    package_name='start_rl_environment'

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Get the Launch config arguments
    number_of_robots = sl.arg('robot_number')
    map_number = sl.arg('map_number')    
    
    # Create a list of x robots
    robots = gen_robot_list(number_of_robots, map_number)

    # Below is all neccessary for multi-robot control within ros name spaces and stuff
    # This implementation uses a package 'simple_launcher' we can include multiple instances 
    # of a launch description() by using sl.imclude() to launch the 'spawn_robot' launch file multiple times for seperate robots
    for robot in robots:
        sl.include(package_name,'spawn_robots.launch.py','launch/',
                   launch_arguments={
                                  'robot_urdf': xacro_file,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_name_prefix': robot['name']+'/',
                                  'robot_namespace': robot['ns'],
                                  'use_sim_time': 'true',
                                  'rotation': TextSubstitution(text=str(robot['rotation'])),
                                  }.items()
            )
    return sl.launch_description()

# Below is the key line which allows us to convert LaunchConfigurations to raw python variables.
generate_launch_description = sl.launch_description(opaque_function= launch_setup)
    