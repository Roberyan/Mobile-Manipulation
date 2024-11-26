import numpy as np
import pybullet as p
from math import atan2, degrees, pi
import time
import traceback
import math 
from scipy.spatial.transform import Rotation
from cachetools import LRUCache, cached
cache = LRUCache(maxsize=10)


import numpy as np


def visualize_points(p, points, radius=0.05, color=[1, 0, 0, 1]):
    """
    Visualize a list of 3D points in PyBullet as spheres.
    
    Args:
        p: PyBullet physics client
        points: List of 3D points to visualize, e.g., [(x1, y1, z1), (x2, y2, z2), ...]
        radius: Radius of each sphere to represent the points
        color: RGBA color of the spheres
    """
    bodies = []
    for point in points:
        # Create a visual sphere at each point
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE, 
            radius=radius, 
            rgbaColor=color
        )
        
        # Create a body with the visual shape at the point location
        bodies.append(p.createMultiBody(
            baseVisualShapeIndex=visual_shape, 
            basePosition=point
        ))
    return bodies

def reorder_vertices(points):
    """
    Reorders a list of 3D points by sorting the points and assigning them
    to the correct vertices for the box.

    The order of vertices for a box should be as follows:
    0: (-x, -y, -z), 1: (+x, -y, -z), 2: (+x, +y, -z), 3: (-x, +y, -z)
    4: (-x, -y, +z), 5: (+x, -y, +z), 6: (+x, +y, +z), 7: (-x, +y, +z)

    Args:
        points: A list of 8 points (each as [x, y, z])

    Returns:
        A list of points reordered to match the above convention
    """
    sorted_points = sorted(points, key=lambda p: (p[2], p[1], p[0]))
    
    # Rearrange the sorted points into corners
    corners = [
        sorted_points[0],  # 0: (-x, -y, -z)
        sorted_points[1],  # 1: (+x, -y, -z)
        sorted_points[3],  # 2: (+x, +y, -z)
        sorted_points[2],  # 3: (-x, +y, -z)
        sorted_points[4],  # 4: (-x, -y, +z)
        sorted_points[5],  # 5: (+x, -y, +z)
        sorted_points[7],  # 6: (+x, +y, +z)
        sorted_points[6],  # 7: (-x, +y, +z)
    ]
    return corners

def visualize_box_from_vertices(p, vertices, color=[0, 0, 1, 0.3]):
    """
    Visualize a box using 6 planes from vertices.
    Each plane is a thin box visual shape.
    
    Args:
        p: PyBullet physics client
        vertices: List of 8 3D points defining the box corners
        color: RGBA color for the box faces
    """
    
    corners = reorder_vertices(vertices)
    corners = np.array(corners)
    # Define faces by vertex indices
    # Assuming vertices are in the following order:
    # 0: (-x, -y, -z), 1: (+x, -y, -z), 2: (+x, +y, -z), 3: (-x, +y, -z)
    # 4: (-x, -y, +z), 5: (+x, -y, +z), 6: (+x, +y, +z), 7: (-x, +y, +z)
    faces = [
        # Front face (-y)
        [0, 1, 5, 4],
        # Back face (+y)
        [2, 3, 7, 6],
        # Left face (-x)
        [0, 3, 7, 4],
        # Right face (+x)
        [1, 2, 6, 5],
        # Bottom face (-z)
        [0, 1, 2, 3],
        # Top face (+z)
        [4, 5, 6, 7]
    ]
    
    bodies = []
    
    for face_vertices_idx in faces:
        # Get the four corners of the face
        face_points = corners[face_vertices_idx]
        
        # Calculate face center
        face_center = np.mean(face_points, axis=0)
        
        # Calculate face dimensions
        v1 = face_points[1] - face_points[0]  # First edge
        v2 = face_points[3] - face_points[0]  # Second edge
        
        # Calculate face extents
        extent1 = np.linalg.norm(v1) / 2
        extent2 = np.linalg.norm(v2) / 2
        
        # Determine which axis this face is perpendicular to
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)
        
        # Create visual shape based on face orientation
        if abs(normal[0]) > 0.9:  # Face is perpendicular to x-axis
            visual_shape = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.001, extent1, extent2],
                rgbaColor=color
            )
        elif abs(normal[1]) > 0.9:  # Face is perpendicular to y-axis
            visual_shape = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[extent1, 0.001, extent2],
                rgbaColor=color
            )
        else:  # Face is perpendicular to z-axis
            visual_shape = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[extent1, extent2, 0.001],
                rgbaColor=color
            )
        
        # Create body for the face
        body = p.createMultiBody(
            baseVisualShapeIndex=visual_shape,
            basePosition=face_center
        )
        bodies.append(body)
    
    return bodies



def visualize_aabb_filled(p, object_aabb, is_2d=False, color=[0, 0, 1, 0.3]):  # last value in color is transparency
    visual_shapes = []
    
    # Get AABB corners
    aabb_min, aabb_max = object_aabb
    
    # Compute the center and extent (half-sizes) of the AABB box
    center = [(aabb_min[i] + aabb_max[i]) / 2 for i in range(3)]
    extent = [(aabb_max[i] - aabb_min[i]) / 2 for i in range(3)]
    
    if is_2d:
        # For 2D, we assume that the visualization occurs on the X-Y plane
        # Create a single 2D box to represent the AABB (just a rectangle in the X-Y plane)
        visual_shape = p.createVisualShape(
            p.GEOM_BOX, 
            halfExtents=[extent[0], extent[1], 0.0],  # Thickness of the rectangle is small (0.01)
            rgbaColor=color
        )
        # Position the shape in the center of the AABB
        visual_shapes.append(p.createMultiBody(baseVisualShapeIndex=visual_shape, basePosition=[center[0], center[1], 0]))
    
    else:
        # For 3D, create faces for each side of the AABB
        face_boxes = []
        
        # X-axis planes
        face_boxes.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, extent[1], extent[2]], rgbaColor=color))
        face_boxes.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, extent[1], extent[2]], rgbaColor=color))
        
        # Y-axis planes
        face_boxes.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[extent[0], 0.01, extent[2]], rgbaColor=color))
        face_boxes.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[extent[0], 0.01, extent[2]], rgbaColor=color))
        
        # Z-axis planes
        face_boxes.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[extent[0], extent[1], 0.01], rgbaColor=color))
        face_boxes.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[extent[0], extent[1], 0.01], rgbaColor=color))
        
        # Add each visual shape at the corresponding position
        visual_shapes.append(p.createMultiBody(baseVisualShapeIndex=face_boxes[0], basePosition=[aabb_min[0], center[1], center[2]]))
        visual_shapes.append(p.createMultiBody(baseVisualShapeIndex=face_boxes[1], basePosition=[aabb_max[0], center[1], center[2]]))
        visual_shapes.append(p.createMultiBody(baseVisualShapeIndex=face_boxes[2], basePosition=[center[0], aabb_min[1], center[2]]))
        visual_shapes.append(p.createMultiBody(baseVisualShapeIndex=face_boxes[3], basePosition=[center[0], aabb_max[1], center[2]]))
        visual_shapes.append(p.createMultiBody(baseVisualShapeIndex=face_boxes[4], basePosition=[center[0], center[1], aabb_min[2]]))
        visual_shapes.append(p.createMultiBody(baseVisualShapeIndex=face_boxes[5], basePosition=[center[0], center[1], aabb_max[2]]))
    
    return visual_shapes
    
def remove_visual_shapes(p, visual_shapes):
    """
    Removes all the visual shapes stored in the visual_shapes list
    """
    for visual_id in visual_shapes:
        p.removeBody(visual_id)  # Remove the body from the simulation
    visual_shapes.clear()  # Clear the list of visual shapes

def checkObject2DSize(obj_id):
    m, M = getAABB(obj_id)
    return (M-m)[:-1]


def get_robot_base_pose(p, robot_id, verbose=False):
    # base_link_index
    link_index = 3
    link_state = p.getLinkState(robot_id, link_index)
    link_position = link_state[0]
    link_orientation = link_state[1]
    euler_orientation = p.getEulerFromQuaternion(link_orientation)

    if verbose:
        print("Link Position: ", link_position)
        print("Link Orientation (quaternion): ", link_orientation)
        print("Link Orientation (Euler angles): ", euler_orientation)

    return link_position, link_orientation, euler_orientation

def get_robot_ee_pose(p, robot_id, verbose=False):
    # left gripper index
    link_index = 18
    link_state = p.getLinkState(robot_id, link_index)
    link_position = link_state[0]
    link_orientation = link_state[1]
    euler_orientation = p.getEulerFromQuaternion(link_orientation)

    if verbose:
        print("Link Position: ", link_position)
        print("Link Orientation (quaternion): ", link_orientation)
        print("Link Orientation (Euler angles): ", euler_orientation)

    return link_position, link_orientation, euler_orientation

def get_joint_index_by_name(robot, joint_name):
    num_joints = p.getNumJoints(robot)
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot, i)
        if joint_info[1].decode("utf-8") == joint_name:
            print(f"joint name: {joint_name} found at index {i}")
            return i
    return None  # Return None if the joint name is not found

def get_link_index_by_name(robot, link_name):
    num_joints = p.getNumJoints(robot)  # This gives the number of joints, which is one less than the number of links
    
    for i in range(num_joints):
        link_info = p.getJointInfo(robot, i)
        link_name_in_urdf = link_info[12].decode("utf-8")  # Link name is stored at index 12
        if link_name_in_urdf == link_name:
            print(f"link name: {link_name_in_urdf} found at index {i}")
            return i
    return None  # Return None if the link name is not found

def getLinkInfo(object_id):
    numJoint = p.getNumJoints(object_id)
    LinkList = ['base']
    for jointIndex in range(numJoint):
      jointInfo = p.getJointInfo(object_id, jointIndex)
      link_name = jointInfo[12]
      if link_name not in LinkList:
        LinkList.append(link_name)
    return LinkList

def getNumLinks(object_id):
    return len(getLinkInfo(object_id))

def get_mug_pose(p, mug_id=21):
    position = p.getBasePositionAndOrientation(mug_id)[0]
    return position

def getAABB(object_id):
    numLinks = getNumLinks(object_id)
    AABB_List = []
    for link_id in range(-1, numLinks - 1):
        AABB_List.append(p.getAABB(object_id, link_id))
    AABB_array = np.array(AABB_List)
    AABB_obj_min = np.min(AABB_array[:, 0, :], axis=0)
    AABB_obj_max = np.max(AABB_array[:, 1, :], axis=0)
    AABB_obj = np.array([AABB_obj_min, AABB_obj_max])
    
    return AABB_obj

def attach(p, object_id, robot_id, ee_link_index, threshould=0.2):
    obj_position = p.getBasePositionAndOrientation(object_id)[0]
    ee_position = p.getLinkState(robot_id, ee_link_index)[0]

    if np.linalg.norm(np.array(obj_position) - np.array(ee_position)) > threshould:
        print("Object is too far from the gripper")
        return None
    else:
        attached_constraint = p.createConstraint(
            parentBodyUniqueId=robot_id,
            parentLinkIndex=ee_link_index,
            childBodyUniqueId=object_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0],
        )
        print(f"Attached object id {object_id} with end-effector!")

        return attached_constraint
    
def detach(attached_constraint):
    if attached_constraint:
        p.removeConstraint(attached_constraint)
        print("Detached object from the end-effector!")

def motion_planning_test(p, robot_id, target_position):
    current_ee_position, _, _ = get_robot_ee_pose(p, robot_id)
    if np.linalg.norm(np.array(target_position) - np.array(current_ee_position)) < 0.1:
        print("The end-effector is already at the target position!")


def calculate_angle_to_aim(base_position, target_position):
    """
    Calculate precise angle between two points in 3D space
    """
    p1 = np.array(base_position)
    p2 = np.array(target_position)
    
    # Project to XY plane for angle calculation
    diff_vector = p2 - p1
    angle = np.arctan2(diff_vector[1], diff_vector[0])
    return angle

def get_direction_from_position(position):
    x, y, z = position
    yaw = np.arctan2(y, x)
    
    # Calculate pitch (elevation angle)
    pitch = np.arctan2(z, np.sqrt(x**2 + y**2))
    
    # Roll cannot be determined
    roll = None
    
    return roll, pitch, yaw


def get_angle_diff(base, target):
    base_y = get_direction_from_position(base)[2]
    target_y = get_direction_from_position(target)[2]
    return base_y - target_y

def get_target_orientation(p, relative_angle, current_orientation):
    # Get current Euler angles
    current_euler = p.getEulerFromQuaternion(current_orientation)
    
    # Create new Euler angles, maintaining roll and pitch but updating yaw
    new_yaw = current_euler[2] + relative_angle  # Add relative angle to current yaw
    new_euler = [current_euler[0], current_euler[1], new_yaw]  # [roll, pitch, new_yaw]
    
    # Convert Euler angles back to quaternion
    target_orientation = p.getQuaternionFromEuler(new_euler)
    
    return target_orientation

def calculate_rotation_to_90_counterclockwise(p, robot_base_pos, target_arm_pos, current_orientation):
    # Calculate global target angle
    dx = target_arm_pos[0] - robot_base_pos[0]
    dy = target_arm_pos[1] - robot_base_pos[1]
    global_target_angle = math.atan2(dy, dx)
    
    # Add 90 degrees (π/2) counterclockwise offset
    offset_angle = global_target_angle + math.pi/2
    
    # Current robot's yaw
    current_yaw = p.getEulerFromQuaternion(current_orientation)[2]
    
    # Calculate relative rotation needed
    relative_angle = offset_angle - current_yaw
    
    # Normalize to -pi to pi range
    relative_angle = (relative_angle + math.pi) % (2 * math.pi) - math.pi
    
    return relative_angle

def calculate_rotation_angle(p, robot_pos, target_pos, current_orientation, reverse=False):
    """
    Calculate the rotation angle needed to face the target
    
    Args:
    robot_pos (tuple): Current robot position
    target_pos (list): Target object position
    current_orientation (tuple): Current robot orientation quaternion
    
    Returns:
    float: Rotation angle relative to current orientation
    """
    # Calculate vector to target
    dx = target_pos[0] - robot_pos[0]
    dy = target_pos[1] - robot_pos[1]
    
    # Calculate global target angle
    global_target_angle = math.atan2(dy, dx)
    
    # Convert current orientation to Euler angles
    current_euler = p.getEulerFromQuaternion(current_orientation)
    current_yaw = current_euler[2]
    
    angle_towards = global_target_angle - current_yaw
    angle_away = (global_target_angle + math.pi) - current_yaw
    
    # Normalize both angles to -pi to pi range
    angle_towards = (angle_towards + math.pi) % (2 * math.pi) - math.pi
    angle_away = (angle_away + math.pi) % (2 * math.pi) - math.pi
    if reverse:
        return angle_away
    return angle_towards

def smoothly_rotate_arm_to_position(p, robot_id, joint_index, base_position, target_position, max_velocity=0.5):
    """
    Enhanced position-based smooth rotation
    """
    # Calculate initial angle and difference
    #target_position = [-target_position[1], target_position[0], target_position[2]] # base and arm of bot are 90 apart
    robot_base_position, r_orientation, _ = get_robot_base_pose(p, robot_id)
    
    angle_diff = calculate_rotation_to_90_counterclockwise(p, robot_base_position, target_position, r_orientation)
    
    rotation_direction = np.sign(angle_diff)
    # Increase velocity for more definitive movement
    max_velocity = 0.1 
    
    iterations = 0
    max_iterations = 10000
    
    while abs(angle_diff) > 0.06 and iterations < max_iterations: # We need bot to face 90 away from target
        try:
            # More aggressive velocity control
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=joint_index,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=rotation_direction * max_velocity,
                force=1000  # Increased force
            )
            
            # More simulation steps
            for _ in range(10):
                p.stepSimulation()
            
            # Update current state
            current_position, r_orientation, _ = get_robot_base_pose(p, robot_id)
            angle_diff = calculate_rotation_to_90_counterclockwise(p, current_position, target_position, r_orientation)
            iterations += 1
            
            
        
        except Exception as e:
            print(f"Rotation error: {e}")
            break
    
    # Stop joint movement
    p.setJointMotorControl2(
        bodyUniqueId=robot_id,
        jointIndex=joint_index,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=500
    )
    
    print(f"Rotation completed after {iterations} iterations")

def calculate_position_angle(base_position, target_position):
    """ Calculate precise angle between two points in 3D space """ 
    p1 = np.array(base_position) 
    p2 = np.array(target_position) 
    # Project to XY plane for angle calculation 
    diff_vector = p2 - p1 
    angle = np.arctan2(diff_vector[1], diff_vector[0]) 
    return angle

def get_aabb_from_vertices(vertices):
    """
    Calculate Axis-Aligned Bounding Box from 8 vertices of a box.
    
    Args:
        vertices: List of 8 points, where each point is [x, y, z]
                 The order of vertices doesn't matter
    
    Returns:
        tuple: (min_coords, max_coords) where each is [x, y, z]
    """
    # Convert vertices to numpy array for easier computation
    vertices_array = np.array(vertices)
    
    # Get min and max along each axis
    min_coords = np.min(vertices_array, axis=0)
    max_coords = np.max(vertices_array, axis=0)
    
    return min_coords.tolist(), max_coords.tolist()

def get_vertices_from_aabb(aabb_min, aabb_max):
    """
    Convert an Axis-Aligned Bounding Box (AABB) to vertices in the order:
    0: (-x, -y, -z)    1: (+x, -y, -z)    2: (+x, +y, -z)    3: (-x, +y, -z)
    4: (-x, -y, +z)    5: (+x, -y, +z)    6: (+x, +y, +z)    7: (-x, +y, +z)
    
    Args:
        min_point: numpy array or list [x_min, y_min, z_min]
        max_point: numpy array or list [x_max, y_max, z_max]
    
    Returns:
        vertices: numpy array of shape (8, 3) containing the box vertices
    """
    x_min, y_min, z_min = aabb_min
    x_max, y_max, z_max = aabb_max
    
    vertices = np.array([
        [x_min, y_min, z_min],  # 0
        [x_max, y_min, z_min],  # 1
        [x_max, y_max, z_min],  # 2
        [x_min, y_max, z_min],  # 3
        [x_min, y_min, z_max],  # 4
        [x_max, y_min, z_max],  # 5
        [x_max, y_max, z_max],  # 6
        [x_min, y_max, z_max]   # 7
    ])
    
    return vertices


def translate_aabb(aabb,
                  current_base_pos,
                  target_base_pos):
    """
    Translate an AABB from current base position to target base position
    
    Args:
        aabb: Tuple of (mins, maxs) each (3,) arrays
        current_base_pos: Current base position (3,)
        target_base_pos: Target base position (3,)
    
    Returns:
        Tuple of (mins, maxs) in target position
    """
    mins, maxs = aabb
    target_base_pos = np.array(target_base_pos)
    current_base_pos = np.array(current_base_pos)
    # Calculate the translation vector
    translation = target_base_pos - current_base_pos
    
    # Simply add the translation to both mins and maxs
    new_mins = mins + translation
    new_maxs = maxs + translation
    
    return new_mins.tolist(), new_maxs.tolist()

def get_aabb_top_points(aabb_min, aabb_max):
    """
    Get the four points of the top plane of an AABB.
    Points are returned in counter-clockwise order starting from front-left.
    
    Args:
        aabb_min: [x_min, y_min, z_min]
        aabb_max: [x_max, y_max, z_max]
    
    Returns:
        List of 4 points [x, y, z] forming the top plane
    """
    z = aabb_max[2]  # Use the maximum z-coordinate (top plane)
    
    # Get the four points in counter-clockwise order
    top_points = [
        [aabb_min[0], aabb_min[1], z],  # Front-left
        [aabb_max[0], aabb_min[1], z],  # Front-right
        [aabb_max[0], aabb_max[1], z],  # Back-right
        [aabb_min[0], aabb_max[1], z]   # Back-left
    ]
    
    return top_points

def get_combined_aabb(body_id_1, body_id_2):
    """
    Calculate combined Axis-Aligned Bounding Box (AABB) for two objects in PyBullet.
    
    Args:
        body_id_1: PyBullet body ID of first object
        body_id_2: PyBullet body ID of second object
        
    Returns:
        tuple: (min_coords, max_coords) where each is [x, y, z]
    """
    # Get AABB for first object
    aabb_min_1, aabb_max_1 = p.getAABB(body_id_1)
    
    # Get AABB for second object
    aabb_min_2, aabb_max_2 = p.getAABB(body_id_2)
    
    # Convert to numpy arrays for easier computation
    aabb_min_1 = np.array(aabb_min_1)
    aabb_max_1 = np.array(aabb_max_1)
    aabb_min_2 = np.array(aabb_min_2)
    aabb_max_2 = np.array(aabb_max_2)
    
    # Calculate combined AABB
    combined_min = np.minimum(aabb_min_1, aabb_min_2)
    combined_max = np.maximum(aabb_max_1, aabb_max_2)
    
    return combined_min.tolist(), combined_max.tolist()



def get_aabb_center(aabb_min, aabb_max):
    """
    Calculate the center point of an AABB.
    
    Args:
        aabb_min: [x_min, y_min, z_min]
        aabb_max: [x_max, y_max, z_max]
    
    Returns:
        [x_center, y_center, z_center]
    """
    center = [
        (aabb_min[0] + aabb_max[0]) / 2,  # x center
        (aabb_min[1] + aabb_max[1]) / 2,  # y center
        (aabb_min[2] + aabb_max[2]) / 2   # z center
    ]
    return center

def transform_point(point, 
                    current_base_pos,
                    current_base_orn,
                    target_base_pos,
                    target_base_orn):
    """
    Transform a point from current world position to target world position
    
    Args:
        point: Point in world frame (3,)
        current_base_pos: Current base position (3,)
        current_base_orn: Current base orientation as quaternion (4,) [x,y,z,w]
        target_base_pos: Target base position (3,)
        target_base_orn: Target base orientation as quaternion (4,) [x,y,z,w]
    
    Returns:
        Point in new world position (3,)
    """
    # Calculate relative transformation
    current_rot = Rotation.from_quat(current_base_orn)
    target_rot = Rotation.from_quat(target_base_orn)
    
    # Get relative rotation
    relative_rot = target_rot * current_rot.inv()
    
    # Transform the point:
    # 1. Express point relative to current base position
    # 2. Apply relative rotation
    # 3. Move to target position
    transformed_point = relative_rot.apply(point - current_base_pos) + target_base_pos
    
    return transformed_point


def transform_points(points, current_base_pos,
                  current_base_orn,
                  target_base_pos,
                  target_base_orn):
    transformed_corners = [
        transform_point(corner, current_base_pos, current_base_orn,
                       target_base_pos, target_base_orn)
        for corner in points
    ]
    return transformed_corners
    
    
def transform_aabb(aabb,
                  current_base_pos,
                  current_base_orn,
                  target_base_pos,
                  target_base_orn):
    """
    Transform an AABB from current base frame to target base frame
    
    Args:
        aabb: Tuple of (mins, maxs) each (3,) arrays
        current_base_pos: Current base position (3,)
        current_base_orn: Current base orientation as quaternion (4,) [x,y,z,w]
        target_base_pos: Target base position (3,)
        target_base_orn: Target base orientation as quaternion (4,) [x,y,z,w]
    
    Returns:
        Tuple of (mins, maxs) in target frame
    """
    mins, maxs = aabb
    
    # Get all 8 corners of the AABB
    corners = np.array([
        [mins[0], mins[1], mins[2]],
        [mins[0], mins[1], maxs[2]],
        [mins[0], maxs[1], mins[2]],
        [mins[0], maxs[1], maxs[2]],
        [maxs[0], mins[1], mins[2]],
        [maxs[0], mins[1], maxs[2]],
        [maxs[0], maxs[1], mins[2]],
        [maxs[0], maxs[1], maxs[2]]
    ])
    
    # Transform each corner
    transformed_corners = np.array([
        transform_point(corner, current_base_pos, current_base_orn,
                       target_base_pos, target_base_orn)
        for corner in corners
    ])
    
    # Get new AABB by taking min/max of transformed corners
    new_mins = np.min(transformed_corners, axis=0)
    new_maxs = np.max(transformed_corners, axis=0)
    
    return new_mins.tolist(), new_maxs.tolist()

def check_aabb_overlap(aabb1, aabb2):
    """
    Check if two AABBs are overlapping.
    
    :param aabb1: First AABB ((min_x, min_y, min_z), (max_x, max_y, max_z))
    :param aabb2: Second AABB ((min_x, min_y, min_z), (max_x, max_y, max_z))
    :return: True if overlapping, False otherwise
    """
    (min1_x, min1_y, min1_z), (max1_x, max1_y, max1_z) = aabb1
    (min2_x, min2_y, min2_z), (max2_x, max2_y, max2_z) = aabb2
    
    # Check for overlap in each dimension
    # If there's no overlap in any dimension, the boxes don't intersect
    x_overlap = max1_x >= min2_x and max2_x >= min1_x
    y_overlap = max1_y >= min2_y and max2_y >= min1_y
    z_overlap = max1_z >= min2_z and max2_z >= min1_z
    
    return x_overlap and y_overlap and z_overlap


@cached(cache)
def get_obj_aabb(p, obj_id):
    return p.getAABB(obj_id)


def get_reverse_quaternion_pybullet(p, current_pos, current_orn):
    """
    Get quaternion for 180-degree rotation using PyBullet.
    """
    rotation_180 = [0, 0, 1, 0]  # 180-degree rotation quaternion
    
    # PyBullet handles the quaternion multiplication
    _, new_quaternion = p.multiplyTransforms(current_pos, current_orn,
                                           current_pos, rotation_180)
    return new_quaternion


def generate_surrounding_points(center_point, distance=0.18):
    """
    Generate 8 points around a given center point in different directions.
    
    Parameters:
    - center_point: The central point (x, y)
    - distance: Distance from the center point
    
    Returns:
    - Dictionary of points in 8 different directions
    """
    # Directions in radians (0 is up, then clockwise)
    directions = {
        'up':          (math.pi/2),
        'up_right':    (math.pi/4),
        'up_left':     (3*math.pi/4),
        'left':        (math.pi),
        'right':       (0),
        'down':        (3*math.pi/2),
        'down_right':  (7*math.pi/4),
        'down_left':   (5*math.pi/4)
    }
    
    # Convert center point to numpy array if it's not already
    center = np.array(center_point)
    
    # Generate points
    points = []
    for direction_name, angle in directions.items():
        # Calculate x and y offsets using trigonometry
        x = center[0] + distance * math.cos(angle)
        y = center[1] + distance * math.sin(angle)
        points.append((x, y))
    
    return points

def find_perpendicular_line_points(points, n_points=100, distance=0.01):
    """
    Find points on a line perpendicular to the given collinear points.
    
    Parameters:
    - points: List of 3 collinear points (a, b, c)
    - n_points: Number of points to generate on the perpendicular line
    - distance: Distance from the reference point
    
    Returns:
    - Perpendicular line direction vector
    - List of n points on the perpendicular line
    """

    
    # Convert points to numpy arrays for easier vector operations
    points = [np.array(p) for p in points]
    
    # Calculate the direction vector of the original line
    original_line_vector = points[1] - points[0]
    
    # Create a perpendicular vector 
    # We'll use a simple 2D rotation for this
    # For 3D, we'd need a more robust method like cross product
    perp_vector = np.array([-original_line_vector[1], original_line_vector[0]])
    
    # Normalize the perpendicular vector
    perp_vector = perp_vector / np.linalg.norm(perp_vector)
    
    # Reference point (can be any of the given points, let's use the first)
    reference_point = points[0]
    
    # Generate n points on the perpendicular line
    perpendicular_points = []
    for i in range(-(n_points//2), n_points//2 + (n_points%2)):
        point = reference_point + i * distance * perp_vector
        perpendicular_points.append(point)
    
    return perpendicular_points





def get_all_obj_aabb(p):
    num_bodies = p.getNumBodies()
    object_ids = []
    for body_id in range(num_bodies):
        object_ids.append(body_id)
    obj_aabbs = {}
    for obj_id in object_ids:
        aabb = get_obj_aabb(p, obj_id)
        obj_aabbs[(obj_id, -1)] = {
            'aabb': aabb,
            'vertices': get_vertices_from_aabb(*aabb)
        }
        if obj_id in [9, 10,11,12]:
            num_joints = p.getNumJoints(obj_id)
            # All other links
            for link_index in range(num_joints):
                aabb = p.getAABB(obj_id, link_index)
                obj_aabbs[(obj_id, link_index)] = {
                    'aabb': aabb,
                    'vertices': get_vertices_from_aabb(*aabb)
                }
    return obj_aabbs