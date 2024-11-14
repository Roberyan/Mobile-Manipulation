from stretch import base_control
import time
import numpy as np
from utils.tools import *
import heapq

class RobotNavigator:
    forward_speed = 0.1
    turn_speed = 0.3
    reverse_move = 0 # odd for reverse move
    base_index = 3
    reverse_index = 6
    
    def __init__(self, p, robot, nav_map, astar_path, aim_obj_id):
        self.p = p
        self.robot = robot
        self.num_links = getNumLinks(self.robot.robotId)
        self.nav_map = nav_map
        self.world_path = [self.map_to_world(x, y) for x, y in astar_path[2:]]
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

        self.collision_free_obj_ids = [self.nav_map.objects_dict['plane'], self.robot.robotId, aim_obj_id]
        self.nav_path_visualize_ids = []
        self.sample_points_ids = []

    def get_robot_base_arm_metric(self):
        base_aabb, arm_aabb = self.nav_map.getAABB(self.robot.robotId)
        self.base_length, self.base_width, self.base_height = base_aabb[1] - base_aabb[0]
         
        # base length is not correct actually, should be the same as self.base_width.
        self.base_length = self.base_width
        
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
    
    def get_current_base_position_2D(self):
        position_front = self.p.getLinkState(self.robot.robotId, self.base_index)[0]
        position_back = self.p.getLinkState(self.robot.robotId, self.reverse_index)[0]
        position = [(position_front[0]+position_back[0])/2, (position_front[1]+position_back[1])/2]
        # if self.reverse_move%2 == 0: 
        #     position = self.p.getLinkState(self.robot.robotId, self.base_index)[0]
        # else:
        #     position = self.p.getLinkState(self.robot.robotId, self.reverse_index)[0]
        return position[0], position[1]
    
    def get_current_arm_position_2D(self):
        _, arm_aabb = self.nav_map.getAABB(self.robot.robotId)
        aabb_min, aabb_max = arm_aabb 
        center = (aabb_min+aabb_max)/2
        return center[0], center[1]
    
    def get_angle_diff(self, target_angle):
        return (target_angle - self.get_current_yaw() + np.pi) % (2 * np.pi) - np.pi
    
    def distance_to_estimating_position(self, aim_tuple):
        current_x, current_y = self.get_current_base_position_2D()
        measure_x, measure_y = aim_tuple
        diff_x, diff_y = (measure_x-current_x), (measure_y-current_y)
        return diff_x, diff_y
    
    # default is current position
    def get_base_aabb(self, aim_tuple=None, zoom=1):
        base_x, base_y = self.get_current_base_position_2D()
        if aim_tuple is not None:
            trans_x, trans_y = self.distance_to_estimating_position(aim_tuple)
            base_x += trans_x
            base_y += trans_y
        half_base_extent = zoom * self.base_width / 2
        base_min = (base_x - half_base_extent, base_y - half_base_extent, self.base_z_range[0])
        base_max = (base_x + half_base_extent, base_y + half_base_extent, self.base_z_range[1])
        return base_min, base_max

    # default is current position
    def get_arm_aabb(self, aim_tuple=None, zoom=1):
        arm_x, arm_y = self.get_current_arm_position_2D()
        if aim_tuple is not None:
            trans_x, trans_y = self.distance_to_estimating_position(aim_tuple)
            arm_x += trans_x
            arm_y += trans_y
        
        arm_half_extent = zoom * self.arm_length / 2  # Assuming the arm is square in cross-section
        arm_min = (arm_x - arm_half_extent, arm_y - arm_half_extent, self.arm_z_range[0])
        arm_max = (arm_x + arm_half_extent, arm_y + arm_half_extent, self.arm_z_range[1])
        return arm_min, arm_max
    
    # get intersection volume of aabbs
    def calculate_intersection_volume(self, aabb1, aabb2):
        # Calculate overlap along each axis (x, y, z)
        x_overlap = max(0, min(aabb1[1][0], aabb2[1][0]) - max(aabb1[0][0], aabb2[0][0]))
        y_overlap = max(0, min(aabb1[1][1], aabb2[1][1]) - max(aabb1[0][1], aabb2[0][1]))
        z_overlap = max(0, min(aabb1[1][2], aabb2[1][2]) - max(aabb1[0][2], aabb2[0][2]))
        
        # Calculate and return the intersection volume
        return x_overlap * y_overlap * z_overlap
    
    def get_aabb_center_symmetry_2D(self, aabb, reference_position):
        aabb_min, aabb_max = aabb
        
        # Calculate the center of the original AABB in 2D (x, y only)
        aabb_center_2d = (
            (aabb_min[0] + aabb_max[0]) / 2,
            (aabb_min[1] + aabb_max[1]) / 2
        )
        
        # Calculate the 2D offset from the AABB center to the reference position
        offset_x = reference_position[0] - aabb_center_2d[0]
        offset_y = reference_position[1] - aabb_center_2d[1]
        
        # Create the symmetric AABB by moving each corner in the x and y directions only
        symmetric_aabb_min = (
            aabb_min[0] + 2 * offset_x,
            aabb_min[1] + 2 * offset_y,
            aabb_min[2]  # z-coordinate remains the same
        )
        symmetric_aabb_max = (
            aabb_max[0] + 2 * offset_x,
            aabb_max[1] + 2 * offset_y,
            aabb_max[2]  # z-coordinate remains the same
        )
        
        # Return the new symmetric AABB
        return symmetric_aabb_min, symmetric_aabb_max
    
    # get overlapping ids
    def get_overlapping_ids(self, aabb):
        aabb_min, aabb_max = aabb
        overlapping_objects = self.p.getOverlappingObjects(aabb_min, aabb_max)
        if overlapping_objects:
            return {obj[0] for obj in overlapping_objects}
        else:
            return set()
    # intersection volume measurement
    def get_intersection_volume_at_position(self, aim_tuple, center_symmetric=False, zoom=1):        
        allowed_ids = set(self.collision_free_obj_ids).union(self.nav_path_visualize_ids)
        try:
            allowed_ids.add(self.exploring_id)
        except:
            pass
        
        if center_symmetric:
            base_min, base_max = self.get_aabb_center_symmetry_2D(self.get_base_aabb(aim_tuple, 1), aim_tuple)
        else:
            base_min, base_max = self.get_base_aabb(aim_tuple, 1)
        
        base_ids = self.get_overlapping_ids((base_min, base_max))
        if not base_ids.issubset(allowed_ids):
            print("Collision detected at base; skipping arm calculation.")
            return float('inf')  # Large number to indicate base collision
        
        if center_symmetric:
            arm_min, arm_max = self.get_aabb_center_symmetry_2D(self.get_arm_aabb(aim_tuple, zoom),aim_tuple)
        else:
            arm_min, arm_max = self.get_arm_aabb(aim_tuple, zoom)
        intersection_volume = 0
        arm_ids = self.get_overlapping_ids((arm_min, arm_max))
        if not arm_ids.issubset(allowed_ids):
            non_allowed_ids = arm_ids - allowed_ids
            intersection_volume = sum(
                self.calculate_intersection_volume(getAABB(obj_id), np.array((arm_min, arm_max)))
                for obj_id in non_allowed_ids
            )
        return intersection_volume
        
    # upgraded collision detection
    def is_collision_free(self):
        allowed_ids = set(self.collision_free_obj_ids).union(self.nav_path_visualize_ids)
        allowed_ids.add(self.exploring_id)
        # base range
        base_min, base_max = self.get_base_aabb()
        
        #check base
        base_overlapping_objects = self.p.getOverlappingObjects(base_min, base_max)
        if base_overlapping_objects:
            base_ids = {obj[0] for obj in base_overlapping_objects}
            if not base_ids.issubset(allowed_ids):
                print("base problem", base_ids)
                return False  # Collision detected in the base area
        
        # arm range
        arm_min, arm_max = self.get_arm_aabb()
        
        # check arm
        arm_overlapping_objects = self.p.getOverlappingObjects(arm_min, arm_max)
        if arm_overlapping_objects:
            arm_ids = {obj[0] for obj in arm_overlapping_objects}
            if not arm_ids.issubset(allowed_ids):
                print("arm problem", arm_ids)
                return False  # Collision detected in the arm area
        
        return True
    
    # check if point is within obj aabb
    def if_within_range(self, obj_id, aim_tuple, zoom=1):
        # special treat for robot base aabb measurement
        if obj_id == self.robot.robotId:
            aabb_min, aabb_max = self.get_base_aabb()
        else:
            aabb_min, aabb_max = getAABB(obj_id)
        
        extent_x = (aabb_max[0] - aabb_min[0]) * zoom
        extent_y = (aabb_max[1] - aabb_min[1]) * zoom

        min_x = aabb_min[0] - (extent_x - (aabb_max[0] - aabb_min[0])) / 2
        max_x = aabb_max[0] + (extent_x - (aabb_max[0] - aabb_min[0])) / 2
        min_y = aabb_min[1] - (extent_y - (aabb_max[1] - aabb_min[1])) / 2
        max_y = aabb_max[1] + (extent_y - (aabb_max[1] - aabb_min[1])) / 2
        
        return min_x <= aim_tuple[0] <= max_x and min_y <= aim_tuple[1] <= max_y
    
    # check if two points are within accept range
    def if_close_enough(self, current_tuple, aim_tuple, special_range=None):
        check_range = special_range if special_range is not None else self.accept_range
        return abs(current_tuple[0]-aim_tuple[0])<= check_range and \
            abs(current_tuple[1]-aim_tuple[1])<=check_range
    
    # forward or reverse move
    def change_mode(self):
        assert (self.forward_speed>0 and self.reverse_move%2 == 0) or \
            (self.forward_speed<0 and self.reverse_move%2 == 1), "Error speed direction and moving mode"
        
        # self.turn_to_angle(self.get_current_yaw()+np.pi)
        self.forward_speed *= -1 
        self.reverse_move += 1
    
    # dummy sampling strategy
    def find_possible_better_alternative(self, aim_tuple):
        return aim_tuple[0]+0.18, aim_tuple[1]
    
    def if_change_mode_allowed(self):
        current_tuple = self.get_current_base_position_2D()
        change_collide = self.get_intersection_volume_at_position(current_tuple, center_symmetric=True, zoom=1.3)
        if change_collide != 0:
            return False
        no_change_collide = self.get_intersection_volume_at_position(current_tuple)
        if no_change_collide >= change_collide:
            return True
        
    # decide move forward or reverse to go, currently dummy judgement
    def mode_decide(self, aim_tuple, zoom=1.5):
        
        change_flag = self.if_change_mode_allowed()

        current_collide_score = self.get_intersection_volume_at_position(aim_tuple, zoom=zoom)
        reverse_collide_score = self.get_intersection_volume_at_position(aim_tuple, center_symmetric=True, zoom=zoom)
        
        if current_collide_score == reverse_collide_score:
            # if forward and reverse move is the same, prefer forward move
            if self.forward_speed < 0 and change_flag:
                self.change_mode()
        elif current_collide_score > reverse_collide_score and change_flag:
            # current mode is not the optimized, change mode
            self.change_mode()
        
        if current_collide_score > 0 and reverse_collide_score > 0:
            aim_tuple = self.find_possible_better_alternative(aim_tuple)
            
        return aim_tuple
        
        #        
        if self.if_within_range(self.nav_map.objects_dict["cabinet"], aim_tuple, 1.7):
            # within the area, use reverse move
            if self.forward_speed > 0:
                self.change_mode()
            return aim_tuple[0]+0.18, aim_tuple[1] # tried offset
        else:
            # outside the are, use forward move
            if self.forward_speed < 0:
                self.change_mode()
            return aim_tuple
    
    def turn_time_estimate(self, aim_x, aim_y):
        if self.reverse_move % 2 != 0:
            print("Rotate direction for reverse moving.")
            reverse_offset = np.pi
        else:
            reverse_offset = 0
        current_x, current_y = self.get_current_base_position_2D()
        initial_angle_to_aim = np.arctan2(aim_y - current_y, aim_x - current_x) + reverse_offset
        initial_angle_diff = self.get_angle_diff(initial_angle_to_aim)
        self.turn_speed = np.sign(initial_angle_diff) * abs(self.turn_speed)
        
        # Set maximum rotation time based on initial angle difference and turn speed
        turn_time_estimate = abs(initial_angle_diff) / abs(self.turn_speed)
        return turn_time_estimate
    
    def turn_to_position(self, aim_x, aim_y):
        # Check if reverse move is active
        if self.reverse_move % 2 != 0:
            print("Rotate direction for reverse moving.")
            reverse_offset = np.pi
        else:
            reverse_offset = 0
        
        
        # Set maximum rotation time based on initial angle difference and turn speed
        turn_time_estimate = self.turn_time_estimate(aim_x, aim_y)
        start_time = time.time()
        
        while True:
            # Get current position and calculate real-time angle to aim
            current_x, current_y = self.get_current_base_position_2D()
            angle_to_aim = np.arctan2(aim_y - current_y, aim_x - current_x) + reverse_offset
            current_angle_diff = self.get_angle_diff(angle_to_aim)

            # Adjust speed for smoother turning as we get closer to the target angle
            if abs(current_angle_diff) < 0.2:
                smooth_turn_speed = self.turn_speed * 0.5
            elif abs(current_angle_diff) < 0.1:
                smooth_turn_speed = self.turn_speed * 0.2
            else:
                smooth_turn_speed = self.turn_speed

            # Stop rotating if within tolerance of target angle
            if abs(current_angle_diff) <= 0.02:
                base_control(self.robot, self.p, forward=0, turn=0)
                print("Aligned with target direction.")
                return True

            # Check if rotation time is too long, preventing redundancy
            if time.time() - start_time > turn_time_estimate:
                base_control(self.robot, self.p, forward=0, turn=0)
                print("Rotation completed due to time limit.")
                return False

            # Rotate the robot at the adjusted speed
            base_control(self.robot, self.p, forward=0, turn=smooth_turn_speed)
            time.sleep(1./240.)  # Step the simulation
            self.p.stepSimulation()

    # prevent arm change during base movement    
    def freeze_arm(self):
        # Step 1: Capture current arm joint positions
        arm_joint_indices = [8, 10, 11, 12, 13, 14, 16]  # Replace with actual indices for your arm joints
        current_positions = [self.p.getJointState(self.robot.robotId, joint)[0] for joint in arm_joint_indices]

        # Step 2: Set arm joints to POSITION_CONTROL with captured positions
        for joint, position in zip(arm_joint_indices, current_positions):
            self.p.setJointMotorControl2(self.robot.robotId, joint, p.POSITION_CONTROL, targetPosition=position, force=1000)

    # follow planned path
    def move_according_to_path(self):
        while self.world_path:
            aim_x, aim_y = self.mode_decide(self.world_path[0])
            self.exploring_id = self.visualize_sampled_points((aim_x, aim_y))

            # real action part
            while True:
                time.sleep(1. / 240.)
                self.p.stepSimulation()
                
                if self.if_within_range(self.robot.robotId, (aim_x, aim_y), 0.5):
                    base_control(self.robot, self.p, forward=0, turn=0)
                    break
                
                # Turn toward the target direction
                self.turn_to_position(aim_x, aim_y)
                
                # if self.if_close_enough((current_x, current_y), (aim_x, aim_y)):
                if self.if_within_range(self.robot.robotId, (aim_x, aim_y)):
                    base_control(self.robot, self.p, forward=0, turn=0)
                    break
                
                print("Moving...")
                base_control(self.robot, self.p, forward=self.forward_speed, turn=0)
                self.freeze_arm()
                
            self.remove_sampled_points(self.exploring_id)
            self.world_path.pop(0)
            self.p.removeBody(self.nav_path_visualize_ids.pop(0))
        
        print("Goal object should be nearby.")

    def sample_and_get_nearest(self, tup_1, tup_2, num_samples=200, max_keep=5, avoid_obj=None):
        x,y = tup_1
        x2, y2 = tup_2
        nearest_heap = []
        
        if avoid_obj is not None:
            aabb_min, aabb_max = self.p.getAABB(avoid_obj)
            aabb_x_min, aabb_y_min, _ = aabb_min
            aabb_x_max, aabb_y_max, _ = aabb_max
        
        for _ in range(num_samples):
            # Generate a random angle and distance within the offset range
            angle = np.random.uniform(0, 2 * np.pi)
            distance = np.random.uniform(self.accept_range * 2, self.accept_range * 3.5)

            # Calculate the sampled point's coordinates
            sample_x = x + distance * np.cos(angle)
            sample_y = y + distance * np.sin(angle)

            # Validate the sample and calculate its combined distance if valid
            if self.if_valid_sample(sample_x, sample_y):
                distance_to_first = (sample_x - x) ** 2 + (sample_y - y) ** 2
                distance_to_second = (sample_x - x2) ** 2 + (sample_y - y2) ** 2
                combined_distance = distance_to_first + distance_to_second

                if avoid_obj is not None:
                    closest_x = min(max(sample_x, aabb_x_min), aabb_x_max)
                    closest_y = min(max(sample_y, aabb_y_min), aabb_y_max)
                    aabb_distance = (sample_x - closest_x) ** 2 + (sample_y - closest_y) ** 2
                    # Adjust combined distance to prioritize points farther from the avoid_obj
                    combined_distance -= aabb_distance  # Subtracting makes points farther from obj more desirable
                
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
        half_extent = self.base_width / 2
        existing_objects = self.p.getOverlappingObjects(
            (sample_x-half_extent, sample_y-half_extent, self.base_z_range[0]),
            (sample_x+half_extent, sample_y+half_extent, self.base_z_range[1])
        )
        existing_objects = {obj[0] for obj in existing_objects}
        
        if not existing_objects.issubset(allowed_ids):
            return False

        print("Available for base to move, checking if fit for robot arm...")
        # check arm then
        arm_z_min, arm_z_max = self.arm_z_range
        for corner_x, corner_y in [
            (sample_x + half_extent, sample_y + half_extent),
            (sample_x + half_extent, sample_y - half_extent),
            (sample_x - half_extent, sample_y + half_extent),
            (sample_x - half_extent, sample_y - half_extent)
        ]:
            # Check for collisions at each corner
            corner_objects = self.p.getOverlappingObjects(
                (corner_x - half_extent, corner_y - half_extent, arm_z_min),
                (corner_x + half_extent, corner_y + half_extent, arm_z_max)
            )
            
            # If no objects at the corner, proceed
            if corner_objects is None:
                return True

            corner_objects = {obj[0] for obj in corner_objects}

            # If the corner objects aren't a subset of allowed IDs, it's not valid
            if not corner_objects.issubset(allowed_ids):
                return False

        # If all corners pass the check, it's a valid sample
        return True

    # visualize sampled collision free points
    def visualize_sampled_points(self, sampled_points):
        def visualize_tuple(tuple):
            if len(tuple) == 2:
                x, y = tuple
                return self.p.createMultiBody(
                    baseVisualShapeIndex=self.sample_point_id,
                    basePosition=(x, y, 0)  # Adjust z-axis for better visibility
                )
            elif len(tuple) == 3:
                return self.p.createMultiBody(
                    baseVisualShapeIndex=self.sample_point_id,
                    basePosition=tuple
                )
        
        if isinstance(sampled_points, list):
            for point in sampled_points:
                self.sample_points_ids.append(visualize_tuple(point))
        elif isinstance(sampled_points, tuple):
            return visualize_tuple(sampled_points)

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
            