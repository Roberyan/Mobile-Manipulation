
import numpy as np
import itertools
import time
import traceback
from utils.tools import *
from simulation.stretch import Robot
from simulation.stretch import LinkStateDetector
import random

def check_no_intersection(cuboid1_points, cuboid2_points):
    """
    Checks if two cuboids do NOT intersect using the Separating Axis Theorem (SAT).

    :param cuboid1_points: List of 8 3D points representing the first cuboid.
    :param cuboid2_points: List of 8 3D points representing the second cuboid.
    :return: True if the cuboids do NOT intersect, False otherwise.
    """
    # Convert points to numpy arrays
    cuboid1_points = np.array(cuboid1_points)
    cuboid2_points = np.array(cuboid2_points)
    
    # Get edges from 8 corner points of the cuboid
    def get_edges(points):
        # Only edges along principal directions
        edges = [
            points[1] - points[0],  # Edge from point 0 to 1
            points[3] - points[0],  # Edge from point 0 to 3
            points[4] - points[0]   # Edge from point 0 to 4
        ]
        return np.array(edges)

    # Get face normals (axes of separation)
    edges1 = get_edges(cuboid1_points)
    edges2 = get_edges(cuboid2_points)
    
    axes = []
    axes.extend(edges1)  # Normals of cuboid1 faces
    axes.extend(edges2)  # Normals of cuboid2 faces
    axes.extend([np.cross(e1, e2) for e1 in edges1 for e2 in edges2])  # Cross products
    
    # Normalize axes and filter out near-zero vectors
    axes = [axis / np.linalg.norm(axis) for axis in axes if np.linalg.norm(axis) > 1e-6]
    
    # Project points of cuboids onto each axis and check for disjoint projections
    def project(points, axis):
        projections = np.dot(points, axis)
        return np.min(projections), np.max(projections)

    for axis in axes:
        min1, max1 = project(cuboid1_points, axis)
        min2, max2 = project(cuboid2_points, axis)
        # Check if projections are disjoint
        if max1 < min2 or max2 < min1:
            return True  # Found a separating axis, cuboids do NOT intersect

    return False

class CollisionChecker:
    def __init__(self, p, mobot: Robot, compressed_states=None, stretched_states=None):
        self.p = p
        self.mobot = mobot
        self.robot_id = mobot.robotId
        self.link_state_detector = LinkStateDetector(p, robotId=self.robot_id)
        self.compresses_link_info = compressed_states
        self.stretched_link_info = stretched_states


    def get_link_info_at_target(self, target_base_position, current_link_info):
        target_link_info = {}
        current_base_position, current_orn, _ = get_robot_base_pose(self.p, self.robot_id)[0]
        for link_idx in current_link_info:
            aabb = current_link_info[link_idx]['aabb']
            vertices = current_link_info[link_idx]['vertices']
            new_aaab = translate_aabb(aabb, current_base_position, target_base_position)
            new_vertices = transform_points(vertices, current_base_position, current_orn, target_base_position, current_orn)
            target_link_info[link_idx] = {
                'aabb': new_aaab,
                'vertices': new_vertices
            }
        return target_link_info

    
    def get_link_info_at_target_with_orientation(self, target_base_position, target_base_ori, current_link_info):
        target_link_info = {}
        current_base_state = current_link_info[self.link_state_detector.base_index]
        current_base_pos = current_base_state['link_pos']
        current_base_orn = current_base_state['link_orientation']
        for link_idx in current_link_info:
            vertices = current_link_info[link_idx]['vertices']
            aabb = current_link_info[link_idx]['aabb']
            new_points = transform_points(vertices, current_base_pos, current_base_orn, 
                                      target_base_position, target_base_ori)
            new_aabb = transform_aabb(aabb, current_base_pos, current_base_orn, 
                                      target_base_position, target_base_ori)
            target_link_info[link_idx] = {
                'vertices': new_points,
                'aabb': new_aabb
            }
        return target_link_info
    
    def is_colliding_vertices(self, target_link_info):
        for link_idx in target_link_info:
            obj_aabbs = self.mobot.obj_aabbs
            for obj_id in obj_aabbs:
                if obj_id[0] == self.mobot.robotId:
                    continue
                obj_vertices = np.array(obj_aabbs[obj_id]['vertices'])
                link_vertices = np.array(target_link_info[link_idx]['vertices'])
                #self.visualize_obj_points(obj_vertices)
                if not check_no_intersection(obj_vertices, link_vertices):
                    return True
        return False
    
    def is_colliding(self, target_link_info, aabb=True, vertices=False):
        if aabb:
            self.visualize_aabb(target_link_info)
            for link_idx in target_link_info:
                overlapping_objects = self.p.getOverlappingObjects(*target_link_info[link_idx]['aabb'])
                if overlapping_objects and len(overlapping_objects) > 0:
                    for object_id, link_id in overlapping_objects:
                        if object_id == self.robot_id or object_id > 22:
                            continue
                        else:
                            return True
        if vertices:
            return self.is_colliding_vertices(target_link_info)
        return False
    
    def is_colliding_object(self, target_link_info, obj_id):
        obj_aabb = get_obj_aabb(self.p, obj_id)

        for link_idx in target_link_info:
            link_aabb = target_link_info[link_idx]['aabb']
            if check_aabb_overlap(link_aabb, obj_aabb):
                return True
        return False

    def get_arm_movement_aabbs(self, target_arm_position, current_link_info):
        movement_aabbs = {}
        for link_idx in self.link_state_detector.arm_movement_indices:
            aabb = current_link_info[link_idx]['aabb']
            aabb_top_points = get_aabb_top_points(*aabb)
            aabb_bottom_points = []
            for point in aabb_top_points:
                aabb_bottom_points.append([point[0], point[1], target_arm_position[2]-0.1])
            new_aabb_points = [*aabb_top_points, *aabb_bottom_points]
            new_aabb = get_aabb_from_vertices(new_aabb_points)
            movement_aabbs[link_idx] = {'aabb': new_aabb}
        return movement_aabbs

    def check_collision_at_position_orientation(self, target_base_position, target_base_ori, target_link_info=None):
        current_link_info = self.link_state_detector.get_current_link_info()
        if target_link_info is None:
            target_link_info = self.get_link_info_at_target_with_orientation(
                target_base_position, target_base_ori, current_link_info)
        return self.is_colliding(target_link_info)
    
    def get_basic_link_info_orientation(self, target_base_position, target_base_ori, current_link_info=None):
        if not current_link_info:
            #current_link_info = self.link_state_detector.get_current_link_info()
            current_link_info = self.mobot.compressed_joint_states
        detector = self.link_state_detector
        new_current_link_info = {
            detector.base_index: current_link_info[detector.base_index],
            detector.vertical_link_index: current_link_info[detector.vertical_link_index],
            detector.top_link_index: current_link_info[detector.top_link_index]
            
        }
        target_link_info = self.get_link_info_at_target_with_orientation(
                target_base_position, target_base_ori, new_current_link_info)
        return target_link_info

    def visualize_aabb(self, target_link_info):
        all_v = []
        for link_idx in target_link_info:
            v_shapes = visualize_aabb_filled(p, target_link_info[link_idx]['aabb'], 
                                color=[random.uniform(0,1), random.uniform(0,1), random.uniform(0,1), 0.3])
            all_v.extend(v_shapes)
            self.p.stepSimulation()
        remove_visual_shapes(self.p, all_v)
    
    def visualize_obj_points(self, points):
        pts = visualize_points(p, points, 
                                color=[random.uniform(0,1), random.uniform(0,1), random.uniform(0,1), 1])
        self.p.stepSimulation()
        time.sleep(1/240)
        time.sleep(2)
        remove_visual_shapes(self.p, pts)

    def visualize_points(self, target_link_info):
        all_v = []
        for link_idx in target_link_info:
            v_shapes = visualize_points(p, target_link_info[link_idx]['vertices'], 
                                color=[random.uniform(0,1), random.uniform(0,1), random.uniform(0,1), 0.3])
            all_v.extend(v_shapes)
            self.p.stepSimulation()
        return all_v
        

    def check_basic_collision_at_position_orientation_object(
            self, target_base_position, target_base_ori, obj_id, current_link_info=None):
        target_link_info = self.get_basic_link_info_orientation(target_base_position, target_base_ori, current_link_info)
        return self.is_colliding_object(target_link_info, obj_id)

    def check_basic_collision_at_position_orientation(self, target_base_position, target_base_ori):
        target_link_info = self.get_basic_link_info_orientation(target_base_position, target_base_ori)
        all_v = self.visualize_points(target_link_info)
        is_colliding =  self.is_colliding(target_link_info, aabb=False, vertices=True)
        if all_v:
            remove_visual_shapes(self.p, all_v)
        return is_colliding
    
    def check_basic_collision_at_position(self, target_base_position):
        current_link_info = self.link_state_detector.get_current_link_info()
        detector = self.link_state_detector
        new_current_link_info = {
            detector.base_index: current_link_info[detector.base_index],
            detector.vertical_link_index: current_link_info[detector.vertical_link_index],
            detector.top_link_index: current_link_info[detector.top_link_index]
            
        }
        target_link_info = self.get_link_info_at_target(
                target_base_position, new_current_link_info)
        self.visualize_points(target_link_info)
        return self.is_colliding(target_link_info, aabb=False, vertices=True)
    

    def check_collision_max_height(self, target_base_position, target_base_orn):
        #Checking if the arm can go to max height
        current_link_info = self.compresses_link_info
        current_ee_pos = get_robot_ee_pose(self.p, self.robot_id)[0]
        arm_max_height_pos = [current_ee_pos[0], current_ee_pos[1], self.mobot.max_height]
        target_link_info = self.get_link_info_at_target_with_orientation(
            target_base_position=target_base_position,
            target_base_ori=target_base_orn,
            current_link_info=current_link_info
        )
        movement_aabbs = self.get_arm_movement_aabbs(arm_max_height_pos, target_link_info)
        return self.is_colliding(movement_aabbs)
        
    def check_collision_max_arm_stretch(self, target_base_position, target_base_orn):
        if not self.stretched_link_info:
            print("Stretched info not found, create object with stretched robot info")
            return None
        current_link_info = self.stretched_link_info
        target_link_info = self.get_link_info_at_target_with_orientation(
            target_base_position=target_base_position,
            target_base_ori=target_base_orn,
            current_link_info=current_link_info
        )
        return self.is_colliding(target_link_info)
    
    def check_collision_arm_movement(self, target_base_position, target_arm_position):
        """
            The arm is always at base while planning and moving. That is followed all through.
            1. Check the arm can reach max height
            2. Check arm can move to  the target arm position
        """

        current_base_state = get_robot_base_pose(self.p, self.robot_id)
        current_base_pos = current_base_state[0]
        current_base_orn = current_base_state[1]
        relative_angle = calculate_rotation_to_90_counterclockwise(self.p, 
                                                                    target_base_position, 
                                                                    target_arm_pos=target_arm_position,
                                                                    current_orientation=current_base_orn)
        target_base_orn = get_target_orientation(self.p, relative_angle, current_base_orn)
        
        
        if self.check_collision_max_height(target_base_position, target_base_orn):
            return False
        
        # TODO: Instead of checking if arm can move down from full stretch, get IK
        # And check with the necessary joint positions alone
        if self.check_collision_max_arm_stretch(target_base_position, target_base_orn):
            return False

        
        current_link_info = self.stretched_link_info
        #current_link_info = self.link_state_detector.get_current_link_info()

        # Bot is simulated to move from current position to new position and orn
        target_link_info = self.get_link_info_at_target_with_orientation(
            target_base_position=target_base_position,
            target_base_ori=target_base_orn,
            current_link_info=current_link_info
        )
        # Get the movement volume in the target orientation
        movement_aabbs = self.get_arm_movement_aabbs(target_arm_position, target_link_info)
        
        # Checking if there is collision for the bot at the target position and orn
        # Checking if there is any collision if the arm moves
        return self.is_colliding(target_link_info) or self.is_colliding(movement_aabbs)
    
    def check_collision_at_position(self, target_base_position):
        # Assuming the base is now at target_base_position
        # The orientation at target_base is assumed to be same as in current_link_info
        """
        Checks that there is no object inside the bounding box of the base
        The arm including end effector
        The vertical mast
        The top of the mast
        """
        current_link_info = self.link_state_detector.get_current_link_info()
        target_link_info = self.get_link_info_at_target(target_base_position, current_link_info)
        return self.is_colliding(target_link_info)
        

class RobotEndEffectorPlanner:
    def __init__(self, p, mobot, objects_dict, movable_joints, target_object_id=None, 
                 max_reachable_distance=None, max_height = None,):
        self.p = p
        self.mobot = mobot
        self.robot_id = mobot.robotId
        self.objects_dict = objects_dict
        self.num_joints = self.p.getNumJoints(self.robot_id)
        self.end_effector_link = 16
        self.movable_joints = movable_joints
        self.target_object_id = target_object_id
        self.max_reachable_distance=max_reachable_distance
        self.max_height=max_height
        self.collision_checker = CollisionChecker(
            self.p, mobot, 
            compressed_states = mobot.compressed_joint_states,
            stretched_states = mobot.stretched_joint_states)



    # def is_object_in_plane(self, object_id, plane_points):
    #     # Get the object's AABB (Axis-Aligned Bounding Box)
    #     aabb_min, aabb_max = self.p.getAABB(object_id)
        
    #     # Get all corner points of the AABB
    #     corners = [
    #         [aabb_min[0], aabb_min[1], aabb_min[2]],
    #         [aabb_min[0], aabb_min[1], aabb_max[2]],
    #         [aabb_min[0], aabb_max[1], aabb_min[2]],
    #         [aabb_min[0], aabb_max[1], aabb_max[2]],
    #         [aabb_max[0], aabb_min[1], aabb_min[2]],
    #         [aabb_max[0], aabb_min[1], aabb_max[2]],
    #         [aabb_max[0], aabb_max[1], aabb_min[2]],
    #         [aabb_max[0], aabb_max[1], aabb_max[2]]
    #     ]
        
    #     # Check if any corner point is inside the plane
    #     return any(
    #         self.is_point_inside_convex_hull(plane_points, corner) 
    #         for corner in corners
    #     )

    # def is_point_inside_convex_hull(self, hull_points, point):
    #     # Create plane equations from the hull points
    #     def plane_equation(p1, p2, p3):
    #         # Calculate normal vector
    #         normal = np.cross(
    #             np.array(p2) - np.array(p1), 
    #             np.array(p3) - np.array(p1)
    #         )
    #         # Normalize the normal
    #         normal = normal / np.linalg.norm(normal)
    #         # Calculate D in Ax + By + Cz + D = 0
    #         D = -np.dot(normal, p1)
    #         return normal, D

    #     # Check if point is on the same side of all plane faces
    #     point = np.array(point)
    #     planes = [
    #         plane_equation(hull_points[0], hull_points[1], hull_points[2]),
    #         plane_equation(hull_points[1], hull_points[2], hull_points[3]),
    #         plane_equation(hull_points[2], hull_points[3], hull_points[0]),
    #         plane_equation(hull_points[3], hull_points[0], hull_points[1])
    #     ]
        
    #     # Check point's position relative to each plane
    #     return all(
    #         # normal is the unit vector perpendicular to the plane
    #         # if dot product of point and normal is close 0, then point is on the plane
    #         abs(np.dot(normal, point) + D) < 0.003 

    #         for normal, D in planes
    #     )

    # # Example usage
    

    # def get_movement_plane(self, base_position, target_position):
    #     # we always pick from up
    #     all_points = [[base_position[0],base_position[1],self.max_height],
    #                   [target_position[0], target_position[1], self.max_height], 
    #                   target_position,
    #                   [base_position[0], base_position[1], target_position[2]]]
        
        
        
    #     return all_points

    # def check_arm_movement_plane(self, base_position, target_position):
    #     """Check if robot collides with any objects"""
    #     all_points = self.get_movement_plane(base_position, target_position)
    #     for obj_name, obj_id in self.objects_dict.items():
    #         if (self.target_object_id == obj_id or self.robot_id == obj_id):
    #             continue
    #         intersect = self.is_object_in_plane(obj_id, all_points)
    #         if intersect:
    #             return True
    #     return False

    # def check_base_block(self, base_position):
    #     pass
    # def check_fixed_vertical_link(self, base_position):
    #     pass

    # def check_collision(self, base_position, target_position):
    #     collisions = [
    #         self.check_arm_movement_plane(base_position, target_position),
    #         self.check_base_block(base_position),
    #         self.check_fixed_vertical_link(base_position)
    #     ]
    #     return not any(collisions)
 
    def generate_search_configurations(self, target_point, distance_to_target, max_num=2):
        """
        Generate random points within a circle centered at target_point with given radius in XY plane.
        Points maintain the same Z coordinate as current_point.
        Returns points sorted by distance from current_point.
        
        Args:
            current_point (np.array): Starting point [x, y, z]
            target_point (np.array): Center of circle [x, y, z]
            radius (float): Radius of circle
            num_points (int): Number of random points to generate
            
        Returns:
            np.array: Array of points sorted by distance from current_point
        """
        # Convert points to numpy arrays if they aren't already
        current_point = get_robot_base_pose(self.p, self.robot_id)[0]
        current_point = np.array(current_point)
        target_point = np.array(target_point)
        
        # Generate random angles and distances from center
        theta = np.random.uniform(0, 2*np.pi, max_num)
        # Subtracting a constant so the radius is always less than the maximum
        r = np.random.uniform(0, distance_to_target, max_num)
        
        # Generate points in XY plane
        x = current_point[0] + r * np.cos(theta)
        y = current_point[1] + r * np.sin(theta)
        z = np.full(max_num, current_point[2])  # Keep same Z as current_point
        
        # Stack coordinates to create points
        points = np.column_stack((x, y, z))
        
        
        # Calculate distances from current_point to all generated points
        distances = np.array([np.linalg.norm(p - current_point) for p in points])
        
        # Sort points by distance
        sorted_indices = np.argsort(distances)
        sorted_points = points[sorted_indices]
        
        return sorted_points
    
    def plan_end_effector_path(self, target_position):
        """
        Plan collision-free path for end effector
        
        Args:
            target_position (list): Target 3D position
            target_orientation (list, optional): Target orientation
            max_attempts (int): Maximum planning attempts
        
        Returns:
            bool: Success of path planning
        """
        possible_end_positions = []
        current_point = get_robot_base_pose(self.p, self.robot_id)[0]
        dist_current_to_target = np.linalg.norm(np.array(current_point[:2]) - np.array(target_position[:2]))
        
        if dist_current_to_target <= self.max_reachable_distance:
            print("Current position is good enough")
            return None
        
        if target_position[2] > self.max_height:
            print("Cannot reach - too high")
            return None
        to_target_dist = self.max_reachable_distance - dist_current_to_target
        
        try:
            # Generate search configurations
            configurations = self.generate_search_configurations(
                target_position, to_target_dist
            )
            
            for base_pos in configurations:
                # Check collision
                if not self.collision_checker.check_collision_arm_movement(base_pos, target_position):
                    possible_end_positions.append(base_pos)
        except Exception as e:
            traceback.print_exc()
        if not possible_end_positions:
            print("Cannot find good collision free point")
            return None
        return possible_end_positions
    
    