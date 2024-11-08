from stretch import base_control
import time
import numpy as np
from utils.tools import *

class RobotNavigator:
    forward_speed = 0.1
    turn_speed = 0.1
    reverse_move = 0
    base_index = 3
    
    def __init__(self, p, robot, nav_map, astar_path):
        self.p = p
        self.robot = robot
        self.num_links = getNumLinks(self.robot.robotId)
        self.nav_map = nav_map
        self.astar_path = astar_path
        self.world_path = [self.map_to_world(x, y) for x, y in astar_path]
        self.get_robot_base_arm_metric()
    
    def get_robot_base_arm_metric(self):
        base_aabb, arm_aabb = self.nav_map.getAABB(self.robot.robotId)
        self.base_length, self.base_width, self.base_height = base_aabb[1] - base_aabb[0]
        self.arm_length, self.arm_width, self.arm_height = arm_aabb[1] - arm_aabb[0]
        self.base_height += base_aabb[0][-1]
        self.arm_height += arm_aabb[0][-1]
        print("========")
    
    def map_to_world(self, map_x, map_y):
        world_x = self.nav_map.x_min + map_x * self.nav_map.grid_resolution
        world_y = self.nav_map.y_min + map_y * self.nav_map.grid_resolution
        return world_x, world_y
    
    def show_path_in_world(self):
        print("Visualize planned path in the bullet simulation")
        n_points = len(self.world_path)
        for i in range(n_points - 1):
            start_point = self.world_path[i]
            end_point = self.world_path[i+1]
            start_point = (start_point[0], start_point[1], 0.0)
            end_point = (end_point[0], end_point[1], 0.0)
            self.p.addUserDebugLine(start_point, end_point, lineColorRGB=[1, 0, 0], lineWidth=5)
            
            # Add a visual sphere at the start point for better visibility
            sphere_radius = self.nav_map.grid_resolution * 0.5  # Make the size proportional to grid resolution
            self.p.createVisualShape(
                shapeType=self.p.GEOM_SPHERE,
                radius=sphere_radius,
                rgbaColor=[0, 1, 0, 1],  # Green color
                visualFramePosition=start_point
            )

            # Optionally, add a sphere at the end point (unless it's the last iteration)
            if i == n_points - 2:
                self.p.createVisualShape(
                    shapeType=self.p.GEOM_SPHERE,
                    radius=sphere_radius,
                    rgbaColor=[0, 0, 1, 1],  # Blue color
                    visualFramePosition=end_point
                )
    
    def is_within_grid_resolution(self, current_tuple, aim_tuple):
        return abs(current_tuple[0]-aim_tuple[0])<=self.nav_map.grid_resolution and \
            abs(current_tuple[1]-aim_tuple[1])<=self.nav_map.grid_resolution
    
    def get_current_yaw(self):
        orientation = self.p.getLinkState(self.robot.robotId, self.base_index)[1]
        _, _, yaw = self.p.getEulerFromQuaternion(orientation)
        return yaw
    
    def get_current_position_2D(self):
        position = self.p.getLinkState(self.robot.robotId, self.base_index)[0]
        return position[0], position[1]
    
    def turn_to_angle(self, target_angle):
        original_yaw = self.get_current_yaw()
        
        if self.reverse_move%2 != 0:
            print("Rotate direction for reverse moving.")
            target_angle += np.pi
            
        if_try_other_direction = False 
           
        while True:
            print("Adjusting to the correct direction.")
            current_yaw = self.get_current_yaw()
            # Normalize to [-pi, pi]
            angle_diff = (target_angle - current_yaw + np.pi) % (2 * np.pi) - np.pi

            if abs(angle_diff) < 0.05:  # Stop turning when close enough to the target angle
                base_control(self.robot, self.p, forward=0, turn=0)
                break
            
            if not self.is_collision_free():
                if if_try_other_direction:
                    if_try_other_direction = False
                    target_angle = original_yaw # go to original direction
                    print("Two directions are all not available, turn to original direction")
                else:
                    if_try_other_direction = True
                    target_angle += 2*np.pi
                    print("Try other rotation direction")

            turn_direction = np.sign(angle_diff)  # Determine clockwise or counterclockwise direction
            base_control(self.robot, self.p, forward=0, turn=turn_direction * self.turn_speed)
            time.sleep(1./240.)  # Step the simulation
            self.p.stepSimulation()
    
    def is_collision_free(self):
        is_collision_free = True
        for link_index in range(-1, self.num_links):  # -1 includes the base
            # Check contact points with all other objects
            contact_points = p.getContactPoints(bodyA=self.robot.robotId, linkIndexA=link_index)
            if contact_points:
                is_collision_free = False
                break
        return is_collision_free
    
    def move_according_to_path(self):
        for aim_x, aim_y in self.world_path:
            while True:
                time.sleep(1./240.)
                self.p.stepSimulation()
                
                if not self.is_collision_free():
                    base_control(self.robot, self.p, forward=0, turn=0)
                    self.reverse_move += 1
                    self.forward_speed*=-1
                    
                    # move back for some free space to adjust
                    reverse_duration = 1
                    start_time = time.time()
                    while time.time() - start_time < reverse_duration:
                        base_control(self.robot, self.p, forward=self.forward_speed, turn=0)
                        time.sleep(1./240.)
                        self.p.stepSimulation()
                    
                    # Stop the robot after moving backward
                    base_control(self.robot, self.p, forward=0, turn=0)
                    
                current_x, current_y = self.get_current_position_2D()
                if self.is_within_grid_resolution((current_x, current_y), (aim_x, aim_y)):
                    base_control(self.robot, self.p, forward=0, turn=0)
                    break
                
                # turn to right direction
                angle_to_aim = np.arctan2(aim_y - current_y, aim_x - current_x)
                self.turn_to_angle(angle_to_aim)
                
                # move after direction is correct
                print("Moving towards the aim postion")
                base_control(self.robot, self.p, forward=self.forward_speed, turn=0)
        
        print("Arrive at aim position")