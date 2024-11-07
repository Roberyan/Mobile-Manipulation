from stretch import base_control
import time
import numpy as np

class RobotNavigator:
    forward_speed = 0.1
    turn_speed = 0.1
    
    def __init__(self, p, robot, nav_map, astar_path):
        self.p = p
        self.robot = robot
        self.nav_map = nav_map
        self.astar_path = astar_path
        self.world_path = [self.map_to_world(x, y) for x, y in astar_path]
        
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
    
    def move_according_to_path(self):
        for aim_x, aim_y in self.world_path:
            while True:
                time.sleep(1./240.)
                self.p.stepSimulation()
                
                position = self.p.getLinkState(self.robot.robotId, 3)[0]
                current_x, current_y = position[0], position[1]
                if self.is_within_grid_resolution((current_x, current_y), (aim_x, aim_y)):
                    base_control(self.robot, self.p, forward=0, turn=0)
                    break
                
                orientation = self.p.getLinkState(self.robot.robotId, 3)[1]
                _, _, yaw = self.p.getEulerFromQuaternion(orientation)
                
                # aim direction difference and normalize to [-pi, pi]
                angle_to_aim = np.arctan2(aim_y - current_y, aim_x - current_x)
                angle_diff = angle_to_aim - yaw
                angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
                
                # first turn to right direction
                if abs(angle_diff) >= 0.1:
                    turn = np.sign(angle_diff) * self.turn_speed
                    base_control(self.robot, self.p, forward=0, turn=turn)
                    continue
                
                # move after direction is correct
                base_control(self.robot, self.p, forward=self.forward_speed, turn=0)