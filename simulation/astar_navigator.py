from stretch import base_control
import time
import numpy as np
from utils.tools import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class RobotNavigator:
    forward_speed = 0.1
    turn_speed = 0.3
    reverse_move = 0
    base_index = 3
    
    def __init__(self, p, robot, nav_map, astar_path):
        self.p = p
        self.robot = robot
        self.num_links = getNumLinks(self.robot.robotId)
        self.nav_map = nav_map
        self.world_path = [self.map_to_world(x, y) for x, y in astar_path]
        self.get_robot_base_arm_metric()
        
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
        while self.world_path:
            aim_x, aim_y = self.world_path.pop(0)

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
                
                # Neither moving forward or backward can approach the aim, need resampling
                if self.reverse_move > 2: 
                    print("Can not approach current position because of 3D collision, activate sampling to find alternative way")
                    next_x, next_y = self.world_path[0]
                    nearby_postition = self.sample_nearby_points(aim_x, aim_y)
                    nearby_postition = self.rank_sampled_points(nearby_postition, aim_x, aim_y, next_x, next_y)
                    self.visualize_sampled_points(nearby_postition)
                    self.remove_sampled_points()
                    self.reverse_move %= 2 # reset
                else:
                    # turn to right direction
                    angle_to_aim = np.arctan2(aim_y - current_y, aim_x - current_x)
                    self.turn_to_angle(angle_to_aim)
                    
                    # move after direction is correct
                    print("Moving towards the aim postion")
                    base_control(self.robot, self.p, forward=self.forward_speed, turn=0)

            self.p.removeBody(self.nav_path_visualize_ids.pop(0))
        print("Arrive at aim position")

    def rank_sampled_points(self, sampled_points, current_x, current_y, next_x, next_y):
        scored_points = []
        for sample_x, sample_y in sampled_points:
            distance_to_current = np.hypot(sample_x - current_x, sample_y - current_y)
            if distance_to_current >= self.base_length:
                continue
            distance_to_next = np.hypot(sample_x - next_x, sample_y - next_y)
            total_score = distance_to_current + distance_to_next
            scored_points.append((sample_x, sample_y, total_score))
        scored_points.sort(key=lambda point: point[2])
        return [(x, y) for x, y, _ in scored_points]
    
    # check if sampled point is available for robot to move
    def if_valid_sample(self, sample_x, sample_y):
        
        allowed_ids = set(self.collision_free_obj_ids).union(self.nav_path_visualize_ids)
        
        # check base first
        base_range_half = max(self.base_height, self.base_width) / 2
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

    # sample possible points near current aim point as alternative points
    def sample_nearby_points(self, cur_aim_x, cur_aim_y, max_samples=100, std_dev=1):
        sampled_points = []
        sampling_radius = max(self.base_length, self.base_width) * 0.4
        for _ in range(max_samples):
            # Generate a random offset within the square area defined by the sampling_radius
            offset_x = np.random.normal(0, std_dev * sampling_radius)
            offset_y = np.random.normal(0, std_dev * sampling_radius)
            
            # Compute the sampled point's coordinates
            sample_x = cur_aim_x + offset_x
            sample_y = cur_aim_y + offset_y
            if self.if_valid_sample(sample_x, sample_y):
                sampled_points.append((sample_x, sample_y))
        return sampled_points
    
    # visualize sampled collision free points
    def visualize_sampled_points(self, sampled_points):
        for point in sampled_points:
            x, y = point
            body_id = self.p.createMultiBody(
                baseVisualShapeIndex=self.sample_point_id,
                basePosition=(x, y, 0)  # Adjust z-axis for better visibility
            )
            # Store the body ID for potential removal
            self.sample_points_ids.append(body_id)

    # remove visualization
    def remove_sampled_points(self, sampled_point_ids=None):
        if sampled_point_ids is None:
            sampled_point_ids = self.sample_points_ids
            
        for body_id in sampled_point_ids:
            self.p.removeBody(body_id)
        
        if sampled_point_ids is None:
            self.sample_points_ids.clear()
            