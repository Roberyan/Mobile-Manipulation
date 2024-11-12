import numpy as np
import pybullet as p

def visualize_aabb_filled(object_aabb, is_2d=False, color=[0, 0, 1, 0.3]):  # last value in color is transparency
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
    
def remove_visual_shapes(visual_shapes):
    """
    Removes all the visual shapes stored in the visual_shapes list
    """
    for visual_id in visual_shapes:
        p.removeBody(visual_id)  # Remove the body from the simulation
    visual_shapes.clear()  # Clear the list of visual shapes

def checkObject2DSize(obj_id):
    m, M = getAABB(obj_id)
    return (M-m)[:-1]

def get_robot_base_pose(p, robot_id):
    # base_link_index
    link_index = 4
    link_state = p.getLinkState(robot_id, link_index)
    link_position = link_state[0]
    link_orientation = link_state[1]
    print("Link Position: ", link_position)
    print("Link Orientation (quaternion): ", link_orientation)

    euler_orientation = p.getEulerFromQuaternion(link_orientation)
    print("Link Orientation (Euler angles): ", euler_orientation)

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

def attach(object_id, robot_id, ee_link_index, threshould=0.2):
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
