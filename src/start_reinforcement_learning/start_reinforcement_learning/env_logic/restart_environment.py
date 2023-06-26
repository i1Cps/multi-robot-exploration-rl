import os
import rclpy
from rclpy.node import Node

import time
import math
import numpy as np
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState
from geometry_msgs.msg import  Point, Pose, Quaternion

class RestartEnvironment():
    def __init__(self, number_of_robots = 3, map_number = 1):
        self.number_of_robots = number_of_robots
        self.map_number = map_number
        # Create publisher to '/set_entity_state' service
        self.set_model_pose = SetModelPose()
        self.true = True
        # Initial robot poses map 1
        map1_robot_pose1 = Pose(position=Point(x=-0.6, y=-10.0, z=0.5), orientation=Quaternion(z=math.sin((math.pi)/8), w=math.cos((math.pi)/8)))
        map1_robot_pose2 = Pose(position=Point(x=1.8, y=-10.0, z=0.5), orientation=Quaternion(z=math.sin((3*math.pi)/8), w=math.cos((3*math.pi)/8)))
        map1_robot_pose3 = Pose(position=Point(x=1.8, y=-1.0, z=0.5), orientation=Quaternion(z=math.sin((5*math.pi)/8), w=math.cos((5*math.pi)/8)))
        map1_robot_pose4 = Pose(position=Point(x=-0.6, y=-1.0, z=0.5), orientation=Quaternion(z=math.sin((-math.pi)/8), w=math.cos((-math.pi)/8)))
        map1_robot_pose5 = Pose(position=Point(x=-0.94, y=-5.0, z=0.5))
        map1_robot_pose6 = Pose(position=Point(x=1.94, y=-5.0, z=0.5), orientation=Quaternion(z=math.sin((-math.pi)/2), w=math.cos((-math.pi)/2)))
        map1_robot_pose7 = Pose(position=Point(x=0.47, y=-3.02, z=0.5), orientation=Quaternion(z=math.sin((-math.pi)/4), w=math.cos((-math.pi)/4)))
        
        # Initial robot poses map 2
        map2_robot_pose1 = Pose(position=Point(x=9.2, y=-3.0, z=0.5), orientation=Quaternion(z=math.sin((3*math.pi)/8), w=math.cos((3*math.pi)/8)))
        map2_robot_pose2 = Pose(position=Point(x=9.2, y=-1.07, z=0.5), orientation=Quaternion(z=math.sin((-3*math.pi)/8), w=math.cos((-3*math.pi)/8)))
        map2_robot_pose3 = Pose(position=Point(x=-0.55, y=-1.2, z=0.5), orientation=Quaternion(z=math.sin((-math.pi)/8), w=math.cos((-math.pi)/8)))
        map2_robot_pose4 = Pose(position=Point(x=-0.8, y=-10.2, z=0.5), orientation=Quaternion(z=math.sin((math.pi)/8), w=math.cos((math.pi)/8)))
        map2_robot_pose5 = Pose(position=Point(x=-0.93, y=-5.1, z=0.5))
        map2_robot_pose6 = Pose(position=Point(x=1.93, y=-5.1, z=0.5), orientation=Quaternion(z=math.sin((math.pi)/2), w=math.cos((math.pi)/2)))
        map2_robot_pose7 = Pose(position=Point(x=1.53, y=-9.9, z=0.5), orientation=Quaternion(z=math.sin((3*math.pi)/8), w=math.cos((3*math.pi)/8)))
        
        # Safe spot to temporily move all robots before replacing them back in the env in map 1
        map1_safe_pose1 = Pose(position=Point(x=-9.0, y=-9.0, z=0.0))
        map1_safe_pose2 = Pose(position=Point(x=-9.0, y=-8.0, z=0.0))
        map1_safe_pose3 = Pose(position=Point(x=-9.0, y=-7.0, z=0.0))
        map1_safe_pose4 = Pose(position=Point(x=-9.0, y=-6.0, z=0.0))
        map1_safe_pose5 = Pose(position=Point(x=-9.0, y=-5.0, z=0.0))
        map1_safe_pose6 = Pose(position=Point(x=-9.0, y=-4.0, z=0.0))
        map1_safe_pose7 = Pose(position=Point(x=-9.0, y=-3.0, z=0.0))
        
        # Safe spots to temporarily move all robots before replacing back in the env in map 2
        map2_safe_pose1 = Pose(position=Point(x=-8.5, y=8.7, z=0.0))
        map2_safe_pose2 = Pose(position=Point(x=-8.5, y=7.7, z=0.0))
        map2_safe_pose3 = Pose(position=Point(x=-8.5, y=6.7, z=0.0))
        map2_safe_pose4 = Pose(position=Point(x=-8.5, y=5.7, z=0.0))
        map2_safe_pose5 = Pose(position=Point(x=-8.5, y=4.7, z=0.0))
        map2_safe_pose6 = Pose(position=Point(x=-8.5, y=3.7, z=0.0))
        map2_safe_pose7 = Pose(position=Point(x=-8.5, y=2.7, z=0.0))
        
        # Initial goal locations [x,y] for map 1
        map1_goal_pose1 = Pose(position=Point(x=-0.9, y=-8.0, z=0.0))
        map1_goal_pose2 = Pose(position=Point(x=2.23, y=-8.1, z=0.0))
        map1_goal_pose3 = Pose(position=Point(x=-1.0, y=-2.9, z=0.0))
        map1_goal_pose4 = Pose(position=Point(x=2.14, y=-2.9, z=0.0))
        
        
        # Initial goal location [x,y] for map 2
        map2_goal_pose1 = Pose(position=Point(x=7.2, y=-1.53, z=0.0))
        map2_goal_pose2 = Pose(position=Point(x=-4.69, y=-1.93, z=0.0))
        map2_goal_pose3 = Pose(position=Point(x=1.31, y=-2.4, z=0.0))
        map2_goal_pose4 = Pose(position=Point(x=0.55, y=-5.98, z=0.0))
        map2_goal_pose5 = Pose(position=Point(x=0.57, y=-8.35, z=0.0))
        self.k = map1_goal_pose1
        self.p = map1_goal_pose3
        
        # List of random robot poses for map 1
        map1_random_robot_poses = [map1_robot_pose1, map1_robot_pose2, map1_robot_pose3,
                                   map1_robot_pose4, map1_robot_pose5, map1_robot_pose6, map1_robot_pose7]
        
        # List of random robot poses for map 2
        map2_random_robot_poses = [map2_robot_pose1, map2_robot_pose2, map2_robot_pose3,
                                   map2_robot_pose4, map2_robot_pose5, map2_robot_pose6, map2_robot_pose7]
        
        # List of safe poses for map 1
        map1_safe_spots = [map1_safe_pose1, map1_safe_pose2, map1_safe_pose3, map1_safe_pose4,
                                map1_safe_pose5, map1_safe_pose6, map1_safe_pose7]

        # List of safe poses for map 2
        map2_safe_spots = [map2_safe_pose1, map2_safe_pose2, map2_safe_pose3, map2_safe_pose4,
                                map2_safe_pose5, map2_safe_pose6, map2_safe_pose7]
        
        # List of random goal poses for map 1
        map1_random_goal_poses = [
            map1_goal_pose1, map1_goal_pose2, map1_goal_pose3, map1_goal_pose4]
        
        # List of random goal poses for map 2
        map2_random_goal_poses = [
            map2_goal_pose1, map2_goal_pose2, map2_goal_pose3, map2_goal_pose4, map2_goal_pose5
        ]
        
        # Random indicies for map 1
        map1_indicies = np.random.choice(
            len(map1_random_robot_poses), number_of_robots, False)
        
        # Random indicies for map 2
        map2_indicies = np.random.choice(
            len(map2_random_robot_poses),  number_of_robots, False)
        
        # Selected random robot poses for map 1
        map1_selected_robot_poses = []
        for i in range(number_of_robots):
            map1_selected_robot_poses.append(
                map1_random_robot_poses[map1_indicies[i]])
        
        # Selected random robot poses for map 2
        map2_selected_robot_poses = []
        for i in range(number_of_robots):
            map2_selected_robot_poses.append(
                map2_random_robot_poses[map2_indicies[i]])
        
        if str(map_number) == '1':   
            self.selected_robot_poses = map1_selected_robot_poses
            self.map_safe_spots = map1_safe_spots
            self.selected_goal_poses = map1_random_goal_poses
        else:
            self.selected_robot_poses = map2_selected_robot_poses
            self.map_safe_spots = map2_safe_spots
            self.selected_goal_poses = map2_random_goal_poses
        
        self.goal_pose_index = np.random.choice(4)
        # Initial selected goal location
        self.current_goal_pose = self.selected_goal_poses[self.goal_pose_index]

    # Resets the goal position to new chosen pose
    def move_goal(self):
        # Access the node that moves goal
        set_goal = self.set_model_pose
        # Randomly pick a different pose for the goal ---- Honestly this is messy but 
        current_index = self.goal_pose_index
        while current_index == self.goal_pose_index:
            current_index = np.random.choice(4)
        self.goal_pose_index = current_index

        self.current_goal_pose = self.selected_goal_poses[self.goal_pose_index]
        name = 'goal_box'
        set_goal.get_logger().info('%%%%%%%%% Moving Goal %%%%%%%%%')
        set_goal.send_request(name, self.current_goal_pose)
        return (self.current_goal_pose.position.x, self.current_goal_pose.position.y)

    # Spawns a new goal entity with chosen pose
    def spawn_goal(self):
        # Start node that spawns goal entity
        spawn_goal = Spawn_Entity()
        spawn_request = SpawnEntity.Request()
        # TODO work on paths
        #goal_sdf_path = '/home/unruly/ros2_ws/src/my_bot/models/goal_box/model.sdf'
        goal_sdf_path = os.path.join(get_package_share_directory('start_rl_environment'),'models','goal_box','model.sdf')
        spawn_request.name = 'goal_box'
        spawn_request.xml = open(goal_sdf_path, 'r').read()
        spawn_request.robot_namespace = 'goal_box'
        
        spawn_request.initial_pose = self.current_goal_pose
              
        spawn_goal.send_request(spawn_request)
        return (self.current_goal_pose.position.x, self.current_goal_pose.position.y)
    
    # Not currently used
    def delete_goal(self):
        delete_goal = Delete_Entity()
        delete_request = DeleteEntity.Request()
        delete_request.name = 'cylinder_g'
        delete_goal.send_request(delete_request)
        

    # Reset the poses of all robots
    def reset_robots(self):
        # Get node thats moves robots
        set_bots = self.set_model_pose
        # First move robots to safe spots
        for i in range(self.number_of_robots):
            name = 'my_bot' + str(i)
            set_bots.send_request(name, self.map_safe_spots[i])
        # Then move them back to there starting position
        time.sleep(1)
        for i in range(self.number_of_robots):
            name = 'my_bot' + str(i)
            set_bots.send_request(name, self.selected_robot_poses[i])


# Changes location of model 
class SetModelPose(Node):
    def __init__(self):
        super().__init__('set_model_pose')
        # Call the set_parameters service of the robot state publisher to update the pose of the robot
        self.cli = self.create_client(
            srv_type=SetEntityState, srv_name="/set_entity_state")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for servicessss %s...')
        # Set the initial pose of the robot

    def send_request(self, model_name, poseee):
        req = SetEntityState.Request()
        req.state._name = model_name
        req.state._pose = poseee
        self.future = self.cli.call_async(req)
        
        rclpy.spin_until_future_complete(self, self.future)

# Spawn an Entity
class Spawn_Entity(Node):
    def __init__(self):
        super().__init__('Spawn_Entity', namespace='ss')
        self.cli = self.create_client(
            srv_type=SpawnEntity, srv_name="/spawn_entity")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, req):
        self.future = self.cli.call_async(req)
        for i in range(30):
            self.get_logger().info(' ')
        self.get_logger().info('%%%%%%%%%% Spawning Goal %%%%%%%%%%')
        #rclpy.spin_until_future_complete(self, self.future)
        self.destroy_node()

# Deletes an Entity
# Not used currently
class Delete_Entity(Node):
    def __init__(self):
        super().__init__('Spawn_Entity')
        self.cli = self.create_client(
            srv_type=DeleteEntity, srv_name="/delete_entity")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    def send_request(self, req):
        self.future = self.cli.call_async(req)
        self.get_logger().info('deleting')
        rclpy.spin_until_future_complete(self, self.future)
        self.destroy_node()
        