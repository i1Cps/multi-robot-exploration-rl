import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import math

# Initializes robot state publisher for each robot, then actually spawns the robot to gazebo world with unique namespace
# For decentralized multi agent control of command velocities and Laser scans 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name')
    x_pos = LaunchConfiguration('x')
    y_pos = LaunchConfiguration('y')
    z_pos = LaunchConfiguration('z')
    turn_around = LaunchConfiguration('rotation')
    robot_ns = LaunchConfiguration('robot_namespace')
    robot_name_prefix = LaunchConfiguration('robot_name_prefix')
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('start_rl_environment'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    assert os.path.exists(xacro_file), "That robot urdf doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # Currently just spawns robot on the first gazebo server it finds running.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_entity', namespace=robot_name,
                        arguments=['-topic', 'robot_description',
                                   '-entity', robot_name,
                                   '-robot_namespace', robot_name, 
                                   '-x',x_pos,
                                   '-y',y_pos,
                                   '-z',z_pos,
                                   '-Y',turn_around,],
                        output='screen')
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    #spawn_entity = Node(package='my_bot', executable='multi_robot_node.py', arguments=[robot_description_config.toxml()], output='screen'),
        
    # Create a robot_state_publisher node
    params = {'frame_prefix':robot_name_prefix,'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        name='robot_state_publisher',
        namespace=robot_name,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        spawn_entity
    ])