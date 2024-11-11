from stretch import base_control
import time
import numpy as np
from utils.tools import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq

class RobotNavigator:
    forward_speed = 0.15
    turn_speed = 0.3
    reverse_move = 0 # odd for reverse move
    base_index = 3
    reverse_index = 6
    
    def __init__(self, p, robot, nav_map, astar_path):
        self.p = p
        self.robot = robot
        self.num_links = getNumLinks(self.robot.robotId)
        self.nav_map = nav_map
        self.world_path = [self.map_to_world(x, y) for x, y in astar_path]
        self.get_robot_base_arm_metric()
        self.accept_range = 0.1
        self.path_point_id = self.p.createVisualShape(
            shapeType=self.p.GEOM_SPHERE,
            radius=self.nav_map.grid_resolution*0.2,
            rgbaColor=[1, 0, 0, 1]  # Bright orange color for start points
        )
        self.sample_point_id = self.p.createVisualShape(
            shapeType=self.p.GEOM_SPHERE,
            radius=self.nav_map.grid_resolution*0.2,
            rgbaColor=[0, 1, 0, 1]  # Bright orange color for start points
        )
        self.exploring_point_id = self.p.createVisualShape(
            shapeType=self.p.GEOM_SPHERE,
            radius=self.nav_map.grid_resolution*0.2,
            rgbaColor=[1, 1, 0, 1]  # Bright orange color for start points
        )

        self.collision_free_obj_ids = [self.nav_map.objects_dict['plane'], self.robot.robotId]
        self.nav_path_visualize_ids = []
        self.sample_points_ids = []
        
    def get_robot_base_arm_metric(self):
        base_aabb, arm_aabb = self.nav_map.getAABB(self.robot.robotId)
        self.base_length, self.base_width, self.base_height = base_aabb[1] - base_aabb[0]
        self.arm_length, self.arm_width, self.arm_height = arm_aabb[1] - arm_aabb[0]
        self.base_z_range = (base_aabb[0][-1], base_aabb[1][-1])
        self.arm_z_range = (arm_aabb[0][-1], arm_aabb[1][-1])
    
    def map_to_world(self, map_x, map_y):
        world_x = self.nav_map.x_min + map_x * self.nav_map.grid_resolution
        world_y = self.nav_map.y_min + map_y * self.nav_map.grid_resolution
        return world_x, world_y
    
    def show_path_in_world(self):
        print("Visualize planned path in the bullet simulation")
        n_points = len(self.world_path)
        
        for i in range(n_points):
            point = self.world_path[i]
            point_position = (point[0], point[1], 0)  # Adjust z for visibility
            
            # Place a sphere at each point along the path for better visibility
            p_id = self.p.createMultiBody(
                baseVisualShapeIndex=self.path_point_id,
                basePosition=point_position
            )
            self.nav_path_visualize_ids.append(p_id)

            # # Draw lines connecting the points to form the path
            # if i < n_points - 1:
            #     next_point = self.world_path[i + 1]
            #     next_point_position = (next_point[0], next_point[1], 0)  # Adjust z for visibility
            #     l_id = self.p.addUserDebugLine(
            #         point_position, 
            #         next_point_position, 
            #         lineColorRGB=[1, 1, 0],  # Red color for the path line
            #         lineWidth=20
            #     )
            #     self.nav_path_visualize_ids.append(l_id)
    
    def get_current_yaw(self):
        orientation = self.p.getLinkState(self.robot.robotId, self.base_index)[1]
        _, _, yaw = self.p.getEulerFromQuaternion(orientation)
        return yaw
    
    def get_current_position_2D(self):
        position_front = self.p.getLinkState(self.robot.robotId, self.base_index)[0]
        position_back = self.p.getLinkState(self.robot.robotId, self.reverse_index)[0]
        position = [(position_front[0]+position_back[0])/2, (position_front[1]+position_back[1])/2]
        # if self.reverse_move%2 == 0: 
        #     position = self.p.getLinkState(self.robot.robotId, self.base_index)[0]
        # else:
        #     position = self.p.getLinkState(self.robot.robotId, self.reverse_index)[0]
        return position[0], position[1]
    
    def get_angle_diff(self, target_angle):
        return (target_angle - self.get_current_yaw() + np.pi) % (2 * np.pi) - np.pi
    
    def escape_collision(self, mode):
        if mode == "turn":
            while not self.is_collision_free():
                base_control(self.robot, self.p, forward=0, turn=self.turn_speed*-1)
        elif mode == "forward":
            while not self.is_collision_free():
                base_control(self.robot, self.p, forward=self.forward_speed*-1, turn=0)
        base_control(self.robot, self.p, forward=0, turn=0)
    
    def turn_to_angle(self, target_angle):
        
        if self.reverse_move % 2 != 0:
            print("Rotate direction for reverse moving.")
            target_angle += np.pi

        initial_angle_diff = self.get_angle_diff(target_angle)
        self.turn_speed = np.sign(initial_angle_diff) * abs(self.turn_speed)
        
        # Estimate the total time needed to turn based on turn speed and angle
        turn_time_estimate = abs(initial_angle_diff) / abs(self.turn_speed)
        start_time = time.time()
        
        tried_alternate_direction = False
        
        while True:
            elapsed_time = time.time() - start_time
            
            if abs(self.get_angle_diff(target_angle)) <= 0.05:
                return True
            
            # Break if the estimated time to complete the turn is reached
            if elapsed_time > turn_time_estimate:
                base_control(self.robot, self.p, forward=0, turn=0)
                print("Rotation completed.")
                return True
            
            if not self.is_collision_free():
                base_control(self.robot, self.p, forward=0, turn=0)
                self.escape_collision("turn")

                # rotate back
                start_back = time.time()
                while time.time()-start_back<elapsed_time:
                    base_control(self.robot, self.p, forward=0, turn=-1 * self.turn_speed)
                    time.sleep(1./240.)
                    self.p.stepSimulation()
                base_control(self.robot, self.p, forward=0, turn=0)
                
                # try another direction
                if not tried_alternate_direction:
                    tried_alternate_direction = True
                    print("Switching to the opposite rotation direction.") 
                    self.turn_speed *= -1
                    # Recalculate the time estimate for the opposite direction
                    angle_diff = self.get_angle_diff(target_angle)
                    turn_time_estimate = abs(angle_diff) / self.turn_speed
                    start_time = time.time()  # Reset time for the new rotation attempt
                else:
                    print("Collision detected in both directions. Stopping rotation.")
                    return False

            # Rotate the robot in the chosen direction
            base_control(self.robot, self.p, forward=0, turn=self.turn_speed)
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
    
    # check if point is within obj aabb
    def if_within_range(self, obj_id, aim_tuple):
        aabb = getAABB(obj_id)
        min_x, min_y, _ = aabb[0]
        max_x, max_y, _ = aabb[1]
        
        return min_x <= aim_tuple[0] <= max_x and min_y<=aim_tuple[1] <= max_y
    
    # check if two points are within accept range
    def if_close_enough(self, current_tuple, aim_tuple, special_range=None):
        check_range = special_range if special_range is not None else self.accept_range
        return abs(current_tuple[0]-aim_tuple[0])<= check_range and \
            abs(current_tuple[1]-aim_tuple[1])<=check_range
    
    def change_mode(self):
        assert (self.forward_speed>0 and self.reverse_move%2 == 0) or \
            (self.forward_speed<0 and self.reverse_move%2 == 1), "Error speed direction and moving mode"
        
        self.forward_speed *= -1 
        self.reverse_move += 1
        
    def move_according_to_path(self):
        while self.world_path:
            aim_x, aim_y = self.world_path[0]
            
            # TODO: check if aim_x and aim_y is suitable
                   
            exploring_id = self.visualize_sampled_points((aim_x, aim_y))
            
            # real action part
            while True:
                time.sleep(1. / 240.)
                self.p.stepSimulation()
                
                # debug use
                if not self.is_collision_free():
                    base_control(self.robot, self.p, forward=0, turn=0)
                    self.escape_collision("forward")
                    current_x, current_y = self.get_current_position_2D()
                    alternative_pos = self.sample_and_get_nearest(
                        current_x, current_y,
                        aim_x, aim_y, 300, 1
                    )
                    self.visualize_sampled_points(alternative_pos)
                    print("-----------------")
                
                current_x, current_y = self.get_current_position_2D()

                if self.if_close_enough((current_x, current_y), (aim_x, aim_y)):
                # if self.if_within_range(self.robot.robotId, (aim_x, aim_y)):
                    base_control(self.robot, self.p, forward=0, turn=0)
                    break
                
                # Turn toward the target direction
                angle_to_aim = np.arctan2(aim_y - current_y, aim_x - current_x)
                self.turn_to_angle(angle_to_aim)
                print("Moving...")
                base_control(self.robot, self.p, forward=self.forward_speed, turn=0)
            
            self.remove_sampled_points(exploring_id)
            if self.if_close_enough((current_x, current_y), self.world_path[0]):
                self.world_path.pop(0)
                self.p.removeBody(self.nav_path_visualize_ids.pop(0))
        
        print("Goal object should be nearby.")

    def sample_and_get_nearest(self, x, y, x2, y2, num_samples=200, max_keep=5):
        nearest_heap = []
        for _ in range(num_samples):
            # Generate a random angle and distance within the offset range
            angle = np.random.uniform(0, 2 * np.pi)
            distance = np.random.uniform(self.accept_range * 2, self.accept_range * 4)

            # Calculate the sampled point's coordinates
            sample_x = x + distance * np.cos(angle)
            sample_y = y + distance * np.sin(angle)

            # Validate the sample and calculate its combined distance if valid
            if self.if_valid_sample(sample_x, sample_y):
                distance_to_first = (sample_x - x) ** 2 + (sample_y - y) ** 2
                distance_to_second = (sample_x - x2) ** 2 + (sample_y - y2) ** 2
                combined_distance = distance_to_first + distance_to_second

                # Use a max heap to keep only the closest `max_keep` points
                heapq.heappush(nearest_heap, (-combined_distance, (sample_x, sample_y)))
                if len(nearest_heap) > max_keep:
                    heapq.heappop(nearest_heap)  # Remove the farthest point

        # Extract points from the heap and return them sorted by proximity
        sorted_points = [point for _, point in sorted(nearest_heap, key=lambda x: -x[0])]
        return sorted_points
    
    # check if sampled point is available for robot to move
    def if_valid_sample(self, sample_x, sample_y):
        
        allowed_ids = set(self.collision_free_obj_ids).union(self.nav_path_visualize_ids)
        
        # check base first
        base_range_half = self.base_width / 2
        existing_objects = self.p.getOverlappingObjects(
            (sample_x-base_range_half, sample_y-base_range_half, self.base_z_range[0]),
            (sample_x+base_range_half, sample_y+base_range_half, self.base_z_range[1])
        )
        existing_objects = {obj[0] for obj in existing_objects}
        
        if not existing_objects.issubset(allowed_ids):
            return False

        print("Available for base to move, checking if fit for robot arm...")
        # check arm then
        arm_z_min, arm_z_max = self.arm_z_range
        arm_length_check_half = max(self.arm_length, self.arm_width) / 2
        arm_width_check_half = min(self.arm_length, self.arm_width) / 2
        arm_center = [
            (sample_x+base_range_half-arm_length_check_half, sample_y),
            (sample_x-base_range_half+arm_length_check_half, sample_y),
            (sample_x,sample_y+base_range_half-arm_length_check_half),
            (sample_x,sample_y-base_range_half+arm_length_check_half)
        ]
        for arm_position in arm_center:
            arm_x, arm_y = arm_position
            if arm_x == sample_x:
                existing_objects = self.p.getOverlappingObjects(
                    (arm_x-arm_width_check_half, arm_y-arm_length_check_half, arm_z_min),
                    (arm_x+arm_width_check_half, arm_y+arm_length_check_half, arm_z_max)
                )
            else:
                existing_objects = self.p.getOverlappingObjects(
                    (arm_x-arm_length_check_half, arm_y-arm_width_check_half, arm_z_min),
                    (arm_x+arm_length_check_half, arm_y+arm_width_check_half, arm_z_max)
                )
            if existing_objects is None:
                return True
            
            existing_objects = {obj[0] for obj in existing_objects}
            if existing_objects.issubset(allowed_ids):
                return True
        return False

    # visualize relative robot aabbs for checking
    def visualize_aabb(self, sample_x, sample_y):
        # Calculate base AABB
        base_range_half = max(self.base_height, self.base_width) / 2
        base_aabb_min = (sample_x - base_range_half, sample_y - base_range_half)
        
        arm_length_check_half = max(self.arm_length, self.arm_width) / 2
        arm_width_check_half = min(self.arm_length, self.arm_width) / 2
        
        # Positions for arm extensions (sides of the base)
        arm_centers = [
            (sample_x + base_range_half - arm_length_check_half, sample_y),  # Right side
            (sample_x - base_range_half+ arm_length_check_half, sample_y),  # Left side
            (sample_x, sample_y + base_range_half - arm_length_check_half),  # Top side
            (sample_x, sample_y - base_range_half + arm_length_check_half)   # Bottom side
        ]
        
        fig, ax = plt.subplots(figsize=(10, 10))

        # Plot base AABB
        ax.add_patch(
            patches.Rectangle(
                base_aabb_min,
                base_range_half * 2,
                base_range_half * 2,
                linewidth=2,
                edgecolor='blue',
                facecolor='none',
                label='Base AABB'
            )
        )
        
        # Plot arm AABBs
        for i, (arm_x, arm_y) in enumerate(arm_centers):
            if arm_x == sample_x:  # Vertical arms (top and bottom)
                arm_aabb_min = (arm_x - arm_width_check_half, arm_y - arm_length_check_half)
                arm_aabb_max = (arm_x + arm_width_check_half, arm_y + arm_length_check_half)
            else:  # Horizontal arms (left and right)
                arm_aabb_min = (arm_x - arm_length_check_half, arm_y - arm_width_check_half)
                arm_aabb_max = (arm_x + arm_length_check_half, arm_y + arm_width_check_half)
            
            ax.add_patch(
                patches.Rectangle(
                    arm_aabb_min,
                    arm_aabb_max[0] - arm_aabb_min[0],
                    arm_aabb_max[1] - arm_aabb_min[1],
                    linewidth=1.5,
                    edgecolor='green',
                    facecolor='none',
                    label=f'Arm AABB {i+1}' # Only show label once
                )
            )
        
        # Center the plot around the sample point
        ax.set_xlim(sample_x - 2 * base_range_half, sample_x + 2 * base_range_half)
        ax.set_ylim(sample_y - 2 * base_range_half, sample_y + 2 * base_range_half)
        
        # Set plot properties
        ax.set_title('AABB Visualization for Base and Arm')
        ax.set_xlabel('X-coordinate')
        ax.set_ylabel('Y-coordinate')
        ax.grid(True)
        ax.legend()
        ax.set_aspect('equal', adjustable='box')
        
        # Show the plot
        plt.show()

    # visualize sampled collision free points
    def visualize_sampled_points(self, sampled_points):
        if isinstance(sampled_points, list):
            for point in sampled_points:
                x, y = point
                body_id = self.p.createMultiBody(
                    baseVisualShapeIndex=self.sample_point_id,
                    basePosition=(x, y, 0)  # Adjust z-axis for better visibility
                )
                # Store the body ID for potential removal
                self.sample_points_ids.append(body_id)
        elif isinstance(sampled_points, tuple):
            x, y = sampled_points
            body_id = self.p.createMultiBody(
                baseVisualShapeIndex=self.exploring_point_id,
                basePosition=(x, y, 0)  # Adjust z-axis for better visibility
            )
            return body_id

    # remove visualization
    def remove_sampled_points(self, sampled_point_ids=None):
        if sampled_point_ids is None:
            sampled_point_ids = self.sample_points_ids
        
        if isinstance(sampled_point_ids, list):
            for body_id in sampled_point_ids:
                self.p.removeBody(body_id)
        elif isinstance(sampled_point_ids, int):
            self.p.removeBody(sampled_point_ids)
            
        if sampled_point_ids is None:
            self.sample_points_ids.clear()
            