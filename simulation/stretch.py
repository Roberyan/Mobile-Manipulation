import math
import os
import argparse
import sys
import time
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import defaultdict
from enum import Enum


import numpy as np
from PIL import Image
import os
from utils.tools import *

sys.path.append('./')


def init_scene(p, mug_random=False):
    root_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../")
    object_dict = {} # store object and its id
    ################ Plane Environment
    plane_id = p.loadURDF(os.path.join(root_dir,"resource/urdf/plane.urdf"), [0, 0, 0])
    plane_texture_id = p.loadTexture(os.path.join(root_dir,"resource/texture/texture1.jpg"))
    p.changeVisualShape(0,-1,textureUniqueId=plane_texture_id)
    object_dict['plane'] = plane_id
    ################ Robot
    mobot_urdf_file = os.path.join(root_dir,"resource/urdf/stretch/stretch.urdf")
    mobot = Robot(pybullet_api=p, start_pos=[-0.8,0.0,0.03], urdf_file=mobot_urdf_file)
    
    for _ in range(30):
        p.stepSimulation()


    ################
    #### table initialization
    table_height = 0.8
    table_width = 1.10 * 2.0
    table_depth = 1.0
    table_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[table_depth / 2.0, table_width / 2.0,
                                                                        table_height / 2.0])
    table_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[table_depth / 2.0, table_width / 2.0,
                                                                           table_height / 2.0])
    mass = 0
    table_id = p.createMultiBody(mass, baseCollisionShapeIndex=table_c, baseVisualShapeIndex=table_v,
                                          basePosition=(table_depth / 2.0 + 0.1, 0.05, table_height / 2.0))
    table_color = [128 / 255.0, 128 / 255.0, 128 / 255.0, 1.0]
    p.changeVisualShape(table_id, -1, rgbaColor=table_color)
    object_dict['table'] = table_id
    ################
    wall_height = 2.2
    wall_width = table_width + 4.0
    wall_depth = 0.02
    wall_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0, wall_height/2.0])
    wall_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0, wall_height/2.0])
    mass = 0
    wall_center_x = p.getAABB(table_id)[1][0] + wall_depth/2.0

    wall_v2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0-0.5, wall_height/2.0])
    wall_c2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_depth/2.0, wall_width/2.0-0.5, wall_height/2.0])

    wall_id = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c2,\
                                             baseVisualShapeIndex=wall_v2,\
                                             basePosition=(wall_center_x, -1.4, wall_height/2.0))
    object_dict['wall'] = wall_id
    wall_id_back = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c,\
                                             baseVisualShapeIndex=wall_v,\
                                             basePosition=(wall_center_x-3.0, -1.9, wall_height/2.0))

    object_dict['wall_back'] = wall_id_back
    wall_id_front = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c,\
                                             baseVisualShapeIndex=wall_v,\
                                             basePosition=(wall_center_x+3.0, -1.9, wall_height/2.0))
    object_dict['wall_front'] = wall_id_front
    wall_width_left = 2.0
    wall_width_right = 6.0
    wall_depth = 0.02
    wall_v_left = p.createVisualShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_left/2.0, wall_height/2.0])
    wall_c_left = p.createCollisionShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_left/2.0, wall_height/2.0])
    wall_v_right = p.createVisualShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_right/2.0, wall_height/2.0])
    wall_c_right = p.createCollisionShape(p.GEOM_BOX,halfExtents=[wall_depth/2.0, wall_width_right/2.0, wall_height/2.0])

    mass = 0
    wall_left_center_x = p.getAABB(table_id)[1][0] - wall_width_left/2.0
    wall_right_center_x = wall_left_center_x
    wall_left_center_y = p.getAABB(table_id)[0][1] - wall_depth/2.0
    wall_right_center_y = p.getAABB(table_id)[1][1] + wall_depth/2.0

    wall_left_id = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c_left,\
                                             baseVisualShapeIndex=wall_v_left,\
                                             basePosition=(wall_left_center_x-1.0, wall_left_center_y-0.92, wall_height/2.0),
                                             baseOrientation=p.getQuaternionFromEuler((0,0,np.pi/2.0)))
    wall_right_id = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c_right,\
                                             baseVisualShapeIndex=wall_v_right,\
                                             basePosition=(wall_right_center_x+1.0, wall_right_center_y, wall_height/2.0),
                                             baseOrientation=p.getQuaternionFromEuler((0,0,-np.pi/2.0)))

    wall_right_id2 = p.createMultiBody(mass,\
                                             baseCollisionShapeIndex=wall_c_right,\
                                             baseVisualShapeIndex=wall_v_right,\
                                             basePosition=(wall_right_center_x+1.0, wall_right_center_y-6.1, wall_height/2.0),
                                             baseOrientation=p.getQuaternionFromEuler((0,0,-np.pi/2.0)))

    wall_color = [204/255.0,242/255.0,255/255.0,1.0]
    p.changeVisualShape(wall_left_id,-1,rgbaColor=wall_color)
    p.changeVisualShape(wall_right_id,-1,rgbaColor=wall_color)
    p.changeVisualShape(wall_id,-1,rgbaColor=wall_color)
    object_dict['wall_left'] = wall_left_id
    object_dict['wall_right'] = wall_right_id
    object_dict['wall_right2'] = wall_right_id2    

    urdf_dir = os.path.join(root_dir,"resource/urdf")

    table_z = p.getAABB(table_id)[1][2]

    cabinet2_position = [-1.5, 0.25, table_z+ 1.5]
    cabinet2_scaling = 0.7
    cabinet2_orientation = p.getQuaternionFromEuler([0, 0, np.pi])
    cabinet2_id = p.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/cabinets/c2/mobility.urdf"),\
                                    useFixedBase=True,
                                    basePosition=cabinet2_position,\
                                    baseOrientation=cabinet2_orientation,\
                                    globalScaling=cabinet2_scaling)

    p.changeVisualShape(cabinet2_id,2,rgbaColor=[0.5,0.5,0.5,1])
    p.changeVisualShape(cabinet2_id,1,rgbaColor=[1,1,1,1])
    p.changeVisualShape(cabinet2_id,3,rgbaColor=[1,1,1,1])
    p.changeVisualShape(cabinet2_id,4,rgbaColor=[0.5,0.5,0.5,1])


    cabinet_center_x = 1.35 #+ (p.getAABB(table_id)[1][0] - p.getAABB(cabinet1_id)[1][0])/2.0
    cabinet_center_y = -1.25#cabinet_width/2.0
    cabinet_center_z = 1.4

    #cabinet1_position = (cabinet_center_x, -cabinet_center_y, cabinet_center_z)
    cabinet2_position = (cabinet_center_x,  cabinet_center_y, cabinet_center_z)
    #p.resetBasePositionAndOrientation(cabinet1_id, cabinet1_position, cabinet1_orientation)
    p.resetBasePositionAndOrientation(cabinet2_id, cabinet2_position, cabinet2_orientation)
    object_dict['cabinet'] = cabinet2_id
    ############################
    #### fridge initialization
    fridge_position = [0.7, -3.22, 0.9]
    fridge_scaling = 1.0
    fridge_orientation = p.getQuaternionFromEuler([0, 0, 0])
    fridge_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/fridges/f1/mobility.urdf"), \
                                    useFixedBase=True, \
                                    basePosition=fridge_position, \
                                    baseOrientation=fridge_orientation, \
                                    globalScaling=fridge_scaling)
    object_dict['fridge'] = fridge_id

    #######
    table_z = p.getAABB(table_id)[1][2]
    drawer_position = [3.84, 0.05,  0.42]
    drawer_scaling = 0.5
    drawer_orientation = p.getQuaternionFromEuler([0, 0, 0])
    drawer_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/drawers/d1/mobility.urdf"), \
                                    basePosition=drawer_position, \
                                    baseOrientation=drawer_orientation, \
                                    globalScaling=drawer_scaling, \
                                    useFixedBase=True)
    object_dict['drawer'] = drawer_id
    #### bed
    #### table initialization
    bed_height = 0.7#0.12 * 2.0
    bed_width = 1.8
    bed_depth = 2.2
    bed_v = p.createVisualShape(p.GEOM_BOX, halfExtents=[bed_depth / 2.0, bed_width / 2.0,
                                                                        bed_height / 2.0])
    bed_c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[bed_depth / 2.0, bed_width / 2.0,
                                                                           bed_height / 2.0])
    mass = 0
    bed_position = (bed_depth / 2.0 + 1.9, -1.45, bed_height / 2.0)
    bed_id = p.createMultiBody(mass, baseCollisionShapeIndex=bed_c, baseVisualShapeIndex=bed_v,
                                          basePosition=bed_position)
    bed_color = [128 / 255.0, 128 / 255.0, 128 / 255.0, 1.0]
    p.changeVisualShape(bed_id, -1, rgbaColor=bed_color)
    object_dict['bed'] = bed_id
    #### microwave initialization
    table_z = p.getAABB(table_id)[1][2]
    microwave_position = [0.35, 0.72, table_z + 0.15]
    microwave_scaling = 0.4
    microwave_orientation = p.getQuaternionFromEuler([0, 0, 0])

    microwave_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/microwaves/7128/mobility.urdf"), \
                                       basePosition=microwave_position, \
                                       baseOrientation=microwave_orientation, \
                                       globalScaling=microwave_scaling, \
                                       useFixedBase=True)

    p.changeVisualShape(microwave_id, 1, rgbaColor=[0.2, 0.2, 0.2, 1], specularColor=[1., 1., 1.])
    p.changeVisualShape(microwave_id, 0, rgbaColor=[0.4, 0.4, 0.4, 1], specularColor=[1., 1., 1.])
    p.changeVisualShape(microwave_id, 2, rgbaColor=[0.5, 0.5, 0.5, 1])
    p.changeVisualShape(microwave_id, 3, rgbaColor=[0.2, 0.2, 0.2, 1])
    p.resetJointState(microwave_id, 1, np.pi/2.0, 0.0)
    object_dict['microwave'] = microwave_id
    #####

    box_position = [2.25, -3.5 , 0.2]
    box_scaling = 0.4
    box_orientation = p.getQuaternionFromEuler([0, 0.0, np.pi / 2.0 + 0])
    box_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/boxes/b4/mobility.urdf"),
                                 basePosition=box_position,
                                 baseOrientation=box_orientation,
                                 globalScaling=box_scaling,
                                 useFixedBase=False,
                                 flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)

    numJoint = p.getNumJoints(box_id)
    box_AABB = p.getAABB(box_id, 0)
    box_height = box_AABB[1][2] - box_AABB[0][2]
    p.resetBasePositionAndOrientation(box_id, box_position, box_orientation)
    bbox = p.getAABB(box_id)
    bbox2 = p.getAABB(box_id, 0)
    p.resetJointState(box_id, 1, 0.9, 0.0)
    for ji in range(numJoint):
        p.setJointMotorControl2(box_id, ji, p.VELOCITY_CONTROL, force=0.5)
    object_dict['box'] = box_id
    ############################
    bottle_position = [drawer_position[0]+0.1, drawer_position[1]+0.1, table_z+0.49]
    bottle_scaling = 0.2
    bottle_orientation = p.getQuaternionFromEuler([np.pi/2.0, 0.0, 0.0])
    bottle_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/bottles/b3/mobility.urdf"),
                                    basePosition=bottle_position,
                                    baseOrientation=bottle_orientation,
                                    useFixedBase=False,
                                    globalScaling=bottle_scaling,
                                    flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)

    obj_friction_ceof = 2000.0
    p.changeDynamics(bottle_id, -1, lateralFriction=obj_friction_ceof)
    p.changeDynamics(bottle_id, -1, rollingFriction=obj_friction_ceof)
    p.changeDynamics(bottle_id, -1, spinningFriction=obj_friction_ceof)

    p.changeDynamics(bottle_id, -1, mass=0.02)
    #p.changeDynamics(bottle_id, -1, linearDamping=20.0)
    #p.changeDynamics(bottle_id, -1, angularDamping=20.0)
    #p.changeDynamics(bottle_id, -1, contactStiffness=0.1, contactDamping=0.1)
    object_dict['bottle'] = bottle_id

    bowl_position = [0.4, -0.3, table_z + 0.15]
    bowl_scaling = 0.2
    bowl_orientation = p.getQuaternionFromEuler([.0, 0.0, 0.0])
    bowl_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/bowls/b1/model.urdf"), \
                                 basePosition=bowl_position, \
                                 baseOrientation=bowl_orientation, \
                                 globalScaling=bowl_scaling, \
                                 useFixedBase=False, \
                                 flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    bowl_AABB = p.getAABB(bowl_id)
    bowl_height = bowl_AABB[1][2] - bowl_AABB[0][2]
    bowl_position[2] = table_z + bowl_height / 2.0
    p.resetBasePositionAndOrientation(bowl_id, bowl_position, bowl_orientation)
    obj_friction_ceof = 4000.0

    p.changeDynamics(bowl_id, -1, lateralFriction=obj_friction_ceof)
    p.changeDynamics(bowl_id, -1, rollingFriction=obj_friction_ceof)
    p.changeDynamics(bowl_id, -1, spinningFriction=obj_friction_ceof)
    p.changeDynamics(bowl_id, -1, mass=0.01)
    #p.changeDynamics(self.bowl_id, -1, linearDamping=20.0)
    #p.changeDynamics(self.bowl_id, -1, angularDamping=20.0)
    #p.changeDynamics(self.bowl_id, -1, contactStiffness=0.9, contactDamping=0.9)
    object_dict['bowl'] = bowl_id
    
    mug_position = (3.4, -2, 2)
    mug_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0, np.pi - np.pi / 2.0])
    mug_scaling = 0.25
    mug_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/mugs/m1/model.urdf"),
                                 useFixedBase=False,
                                 globalScaling=mug_scaling,
                                 basePosition=mug_position,
                                 baseOrientation=mug_orientation)
    p.changeVisualShape(mug_id, -1, rgbaColor=[1, 0, 0, 1])
    obj_friction_ceof = 4000.0
    p.changeDynamics(mug_id, -1, lateralFriction=0.1, spinningFriction=0.1)
    p.changeDynamics(mug_id, -1, mass=0.01)
    object_dict['mug'] = mug_id


    mug_position = [0.75, -0.87, table_z + 0.15]
    mug_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0, np.pi - np.pi / 2.0])
    mug_scaling = 0.25
    mug_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/mugs/m1/model.urdf"),
                                 useFixedBase=False,
                                 globalScaling=mug_scaling,
                                 basePosition=mug_position,
                                 baseOrientation=mug_orientation)
    p.changeVisualShape(mug_id, -1, rgbaColor=[0, 0, 1, 1])
    obj_friction_ceof = 4000.0
    p.changeDynamics(mug_id, -1, lateralFriction=0.1, spinningFriction=0.1)
    p.changeDynamics(mug_id, -1, mass=0.01)
    object_dict['mug_blue'] = mug_id

    basket_position = [-1, -4.69, 0.48]
    basket_scaling = 0.6
    basket_orientation = p.getQuaternionFromEuler([np.pi / 2.0, 0.0, np.pi / 2.0])
    basket_id = p.loadURDF(fileName=os.path.join(urdf_dir, "obj_libs/trashbins/t2/model.urdf"), \
                                    useFixedBase=True,
                                    basePosition=basket_position, \
                                    baseOrientation=basket_orientation, \
                                    globalScaling=basket_scaling)
    p.changeVisualShape(basket_id, -1, rgbaColor=[200 / 255., 179 / 255., 179 / 255., 1])
    obj_friction_ceof = 5000.0
    p.changeDynamics(basket_id, -1, lateralFriction=obj_friction_ceof)
    p.changeDynamics(basket_id, -1, rollingFriction=obj_friction_ceof)
    p.changeDynamics(basket_id, -1, spinningFriction=obj_friction_ceof)
    p.changeDynamics(basket_id, -1, mass=2)
    p.resetBasePositionAndOrientation(basket_id, basket_position, basket_orientation)

    object_dict['basket'] = basket_id

    pan_position = [0.35, .2, table_z + 0.05]
    pan_scaling = 0.6
    pan_orientation = p.getQuaternionFromEuler([.0, 0.0, np.pi / 4.0])
    pan_id = p.loadURDF(os.path.join(urdf_dir, "obj_libs/pans/p1/model.urdf"), \
                                 basePosition=pan_position, \
                                 baseOrientation=pan_orientation, \
                                 globalScaling=pan_scaling, \
                                 useFixedBase=False, \
                                 flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
    p.changeVisualShape(pan_id, 1, rgbaColor=[0.9, 0.9, 0.9, 1.0])
    p.changeDynamics(pan_id, -1, mass=0.001)
    object_dict['pan'] = pan_id

    spatula_position = np.copy(np.array(pan_position))
    spatula_position[1] -= 0.3
    spatula_position[0] += 0.25
    spatula_position[2] += 0.1
    spatula_scaling = 0.4
    spatula_orientation = p.getQuaternionFromEuler([np.pi/2-np.pi/8.,0.,0.])
    spatula_id = p.loadURDF( os.path.join(urdf_dir, "obj_libs/spatula/model.urdf"),\
                                          basePosition=spatula_position,\
                                          baseOrientation=spatula_orientation,\
                                          globalScaling=spatula_scaling,\
                                          flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,useFixedBase=False)

    p.changeVisualShape(spatula_id, -1, rgbaColor=[0 / 255.0, 179 / 255., 179 / 255., 1])
    p.changeDynamics(spatula_id, -1, mass=0.01)
    object_dict['spatula'] = spatula_id

    mug_position = [drawer_position[0]-0.15, drawer_position[1], 1.5]
    mug_orientation = p.getQuaternionFromEuler([np.pi/2.0, 0, np.pi + np.pi/2.0])
    if mug_random:
        mug_position[0] += np.random.uniform(-0.05,0.1)
        mug_position[1] += np.random.uniform(-0.1,0.1)
        mug_orientation = p.getQuaternionFromEuler([np.pi/2.0, 0, np.pi + np.pi/2.0 + np.random.uniform(-np.pi/4.0,np.pi/4.0)])

    mug_scaling = 0.25
    mug_id = p.loadURDF(fileName=os.path.join(urdf_dir,"obj_libs/mugs/m1/model.urdf"),
                                    useFixedBase=False,
                                    globalScaling=mug_scaling,
                                    basePosition=mug_position,
                                    baseOrientation=mug_orientation)
    object_dict['final_mug'] = mug_id
    p.changeVisualShape(mug_id, -1, rgbaColor=[1.0,1.0,1.0,1])
    obj_friction_ceof = 4000.0
    p.changeDynamics(mug_id, -1, lateralFriction=obj_friction_ceof)
    p.changeDynamics(mug_id, -1, mass=0.01)
    # mug id: 22
    
    for _ in range(20):
        p.stepSimulation()

    p.changeVisualShape(mobot.robotId,0,rgbaColor=[1,0,0,1])
    p.changeVisualShape(mobot.robotId,1,rgbaColor=[0,1,0,1])

    p.setRealTimeSimulation(1)

    for j in range (p.getNumJoints(mobot.robotId)):
        print(p.getJointInfo(mobot.robotId,j))
      
    p.resetJointState(drawer_id, 5, 0.3)    # make the first drawer open

    for _ in range(30):
        p.stepSimulation()

    return mobot, object_dict, mug_id, drawer_id


def get_global_action_from_local(p, robot, delta_forward):
    # Get the current joint angle of joint 2 (rotation around z-axis)
    joint2_state = p.getJointState(robot, 3)
    current_yaw = joint2_state[0]  # Get the current rotation (yaw angle)
    
    # Calculate the delta in world coordinates using the yaw angle (rotation around z-axis)
    delta_x = delta_forward * np.cos(current_yaw)  # Change along world x-axis
    delta_y = delta_forward * np.sin(current_yaw)  # Change along world y-axis
    
    return delta_x, delta_y

def base_control(robot, p, forward=0, turn=0):
    x_forward, y_forward = get_global_action_from_local(p, robot.robotId, forward)
    p.setJointMotorControl2(robot.robotId,3,p.VELOCITY_CONTROL,targetVelocity=turn,force=1000)
    p.setJointMotorControl2(robot.robotId,1,p.VELOCITY_CONTROL,targetVelocity=x_forward,force=1000)
    p.setJointMotorControl2(robot.robotId,2,p.VELOCITY_CONTROL,targetVelocity=y_forward,force=1000)
    time.sleep(1. / 240.)
    p.stepSimulation()
    
def arm_control(robot, p, up=0, stretch=0, roll=0, yaw=0):
    # up and down
    p.setJointMotorControl2(robot.robotId,8,p.VELOCITY_CONTROL,targetVelocity=0.2*up,force=1000)

    # stretch and shrink
    p.setJointMotorControl2(robot.robotId,10,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robotId,11,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robotId,12,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    p.setJointMotorControl2(robot.robotId,13,p.VELOCITY_CONTROL,targetVelocity=0.1*stretch,force=100)
    
    # rotate
    p.setJointMotorControl2(robot.robotId,14,p.VELOCITY_CONTROL,targetVelocity=roll,force=1000)
    p.setJointMotorControl2(robot.robotId,16,p.VELOCITY_CONTROL,targetVelocity=yaw,force=1000)
    
def gripper_control(mobot, p, cmd=0):
    # 1 is open, 0 is close
    p.setJointMotorControl2(mobot.robotId,18,p.VELOCITY_CONTROL,targetVelocity=-cmd,force=1000)      # joint left finger
    p.setJointMotorControl2(mobot.robotId,19,p.VELOCITY_CONTROL,targetVelocity=cmd,force=1000)    # joint right gripper


class LinkStateDetector:
    def __init__(self, p, robotId):
        # When this function is called the bot is facing the positive x axis
        self.p = p
        self.robotId = robotId
        self.end_effector_index = 17
        self.base_index = 3
        self.vertical_link_index = 7
        self.lift_link_index = 8
        self.arms_indices = [9, 10, 11, 12, 13]
        self.top_link_index = 20
        self.collision_link_indices = [
            self.base_index,
            self.end_effector_index,
            self.vertical_link_index,
            self.lift_link_index,
            *self.arms_indices,
            self.top_link_index
        ]
        self.arm_movement_indices = [
            self.lift_link_index,
            *self.arms_indices,
            self.end_effector_index
        ]
    
    def get_current_link_info(self):
        link_indices = self.collision_link_indices
        link_info = {}
        for idx in link_indices:
            aabb = self.p.getAABB(self.robotId, idx)
            centre_pos = get_aabb_center(*aabb)
            link_state_idx = self.p.getLinkState(self.robotId, idx)
            pos = link_state_idx[0]
            ori = link_state_idx[1]
            link_info[idx] = {
                'aabb_centre_position': centre_pos,
                'aabb': aabb,
                'link_pos':pos,
                'link_orientation':ori,
                'vertices': get_vertices_from_aabb(*aabb)
            }
        return link_info


class Robot:
    def __init__(self,pybullet_api,start_pos=[0.4,0.3,0.4],urdf_file=None,resource_dir=None,project_root_dir=None):
        self.p = pybullet_api

        self.gripperMaxForce = 1000.0
        self.armMaxForce = 200.0
        self.robot_threshold = 1.0 # distinguish base and arm
        self.start_pos = start_pos
        self.camera_index = 13
        self.end_effector_idx = 17
        self.left_finger_index = 18
        self.right_finger_index = 19
        self.max_reachable_distance = None
        self.max_height = None
        self.original_joint_positions = None
        self.project_dir = project_root_dir
        self.resource_dir = resource_dir
        self.urdf_file = urdf_file
        self.robotId = self.p.loadURDF(self.urdf_file, self.start_pos, useFixedBase=True)
        self.p.resetBasePositionAndOrientation(self.robotId, self.start_pos, [0, 0, 0, 1])
        #  self.p.resetJointState(self.robotId, self.camera_index, -0.3)
        self.p.resetJointState(self.robotId, 4, 0.5)
        self.movable_joints = self.get_movable_joints()
        self.link_state_detector = LinkStateDetector(self.p, self.robotId)
        self.compressed_joint_states = self.link_state_detector.get_current_link_info()
        self.get_max_ee_reach(False)
        self.stretched_joint_states = self.link_state_detector.get_current_link_info()
        self.move_arm_joints_to_contracted_position()
        self.move_arm_to_max_height()
        self.max_height_joint_states = self.link_state_detector.get_current_link_info()
        self.move_arm_joints_to_contracted_position()
        self.obj_aabbs = get_all_obj_aabb(p)

    def get_observation(self):
        camera_link_pos = self.p.getLinkState(self.robotId,self.camera_index)[0]
        camera_link_ori = self.p.getLinkState(self.robotId,self.camera_index)[1]
        camera_link_rotmat = self.p.getMatrixFromQuaternion(camera_link_ori)
        camera_link_rotmat = np.array(camera_link_rotmat).reshape((3, 3))
        camera_link_pos -= 0.1*camera_link_rotmat[:,1]
        camera_target_link_pos = np.array(camera_link_pos)
        camera_target_link_pos = camera_target_link_pos - camera_link_rotmat[:,1]

        self.p.changeVisualShape(self.robotId,self.camera_index,rgbaColor=[0,0,1])

        camera_view_matrix = self.p.computeViewMatrix(cameraEyePosition=[camera_link_pos[0], camera_link_pos[1], camera_link_pos[2]],
                                         cameraTargetPosition=[camera_target_link_pos[0], camera_target_link_pos[1], camera_target_link_pos[2]],
                                         cameraUpVector=camera_link_rotmat[:,2])


        ratio = 1.5
        image_width = int(640 * ratio)
        image_height = 480
        #self.p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
        camera_proj_matrix = self.p.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=10)
        #$initAxis(camera_link_pos, camera_link_ori)
        self.p.getCameraImage(width=image_width,
                                      height=image_height,
                                      viewMatrix = camera_view_matrix,
                                      projectionMatrix=camera_proj_matrix,
                                      renderer = self.p.ER_BULLET_HARDWARE_OPENGL)
        
    def save_image(self, filepath, filename):
        camera_link_pos = self.p.getLinkState(self.robotId,self.camera_index)[0]
        camera_link_ori = self.p.getLinkState(self.robotId,self.camera_index)[1]
        camera_link_rotmat = self.p.getMatrixFromQuaternion(camera_link_ori)
        camera_link_rotmat = np.array(camera_link_rotmat).reshape((3, 3))
        camera_target_link_pos = np.array(camera_link_pos)
        camera_target_link_pos = camera_target_link_pos + camera_link_rotmat[:,0]
        camera_color = [0, 0 / 255.0, 0 / 255.0, 1.0]
        self.p.changeVisualShape(self.robotId,self.camera_index,rgbaColor=[0,0,1])

        camera_view_matrix = self.p.computeViewMatrix(cameraEyePosition=[camera_link_pos[0], camera_link_pos[1], camera_link_pos[2]],
                                                    cameraTargetPosition=[camera_target_link_pos[0], camera_target_link_pos[1], camera_target_link_pos[2]],
                                                    cameraUpVector=camera_link_rotmat[:,1])


        ratio = 1.5
        image_width = int(640 * ratio)
        image_height = 480
        #self.p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
        camera_proj_matrix = self.p.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=10)
        #$initAxis(camera_link_pos, camera_link_ori)
        width, height, rgb_img, depth_img, seg_mask = self.p.getCameraImage(width=image_width,
                                                                            height=image_height,
                                                                            viewMatrix = camera_view_matrix,
                                                                            projectionMatrix=camera_proj_matrix,
                                                                            renderer = p.ER_BULLET_HARDWARE_OPENGL)
        rgb_image = Image.fromarray(np.reshape(rgb_img, (height, width, 4))[:, :, :3])
        full_filename = os.path.join(filepath, filename)
        full_image_filename = full_filename + ".png"
        rgb_image.save(full_image_filename)

        # Convert depth data to a 16-bit format and save it as a TIFF
        # Scaling the depth buffer into a 16-bit range
        depth_img = np.reshape(depth_img, (height, width))
        depth_16bit = (depth_img * 65535).astype(np.uint16)
        depth_image = Image.fromarray(depth_16bit)
        full_depth_filename = full_filename + ".tiff"
        depth_image.save(full_depth_filename)
    
    def get_position(self):
        return self.p.getBasePositionAndOrientation(self.robotId)[0]
    
    def getLinkInfo(self):
        numJoint = self.p.getNumJoints(self.robotId)
        LinkList = ['base']
        for jointIndex in range(numJoint):
            jointInfo = self.p.getJointInfo(self.robotId, jointIndex)
            link_name = jointInfo[12]
            if link_name not in LinkList:
                LinkList.append(link_name)
        return LinkList

    def getNumLinks(self):
        return len(self.getLinkInfo())
    
    def getAABB(self):
        numLinks = self.getNumLinks()
        AABB_base = []
        AABB_arm = []
        for link_id in range(-1, numLinks-1):
            aabb = self.p.getAABB(self.robotId, link_id)
            if aabb[1][2] <= self.robot_threshold:
                AABB_base.append(aabb)
            else:
                AABB_arm.append(aabb)
        
        AABB_base_array = np.array(AABB_base)
        AABB_base_min = np.min(AABB_base_array[:, 0, :], axis=0)
        AABB_base_max = np.max(AABB_base_array[:, 1, :], axis=0)
        AABB_base = np.array([AABB_base_min, AABB_base_max])
        
        AABB_arm_array = np.array(AABB_arm)
        AABB_arm_min = np.min(AABB_arm_array[:, 0, :], axis=0)
        AABB_arm_max = np.max(AABB_arm_array[:, 1, :], axis=0)
        AABB_arm = np.array([AABB_arm_min, AABB_arm_max])
        
        return AABB_base, AABB_arm

    def get_movable_joints(self):
        num_joints = self.p.getNumJoints(self.robotId)
        movable_joints = []
        for i in range(num_joints):
            joint_info = self.p.getJointInfo(self.robotId, i)
            if joint_info[2] != self.p.JOINT_FIXED:
                movable_joints.append((i))
                print("index, name", i, joint_info[1])
        self.movable_joints = movable_joints
        return movable_joints
    
    def get_max_ee_reach(self, reset=True):
        robot_id = self.robotId
        p = self.p
        original_joint_positions = {}
        remove_indices = {1, 2, 3, 4, 5}  # Use a set for faster membership tests
        movable_joints = self.movable_joints
        # Use filter to keep only elements that are not in remove_indices
        movable_joints = list(filter(lambda joint: joint not in remove_indices, movable_joints))

        for joint_index in movable_joints:
            joint_state = p.getJointState(robot_id, joint_index)
            original_joint_positions[joint_index] = joint_state[0]  # joint_state[0] is the joint position
        self.original_joint_positions = original_joint_positions
        joint_limits = {}
        for joint_index in movable_joints:
            joint_info = p.getJointInfo(robot_id, joint_index)
            joint_type = joint_info[2]
            
            if joint_type == p.JOINT_PRISMATIC:
                joint_lower_limit = p.getJointInfo(robot_id, joint_index)[8]  # Lower joint limit (min distance)
                joint_upper_limit = p.getJointInfo(robot_id, joint_index)[9]  # Upper joint limit (max distance)
                joint_limits[joint_index] = (joint_lower_limit, joint_upper_limit)
        
        for joint_index in movable_joints:
            # Set joint to its max limit
            if joint_index in joint_limits:
                min_limit, max_limit = joint_limits[joint_index]
                # For revolute joints, set the joint to its max angle
                if p.getJointInfo(robot_id, joint_index)[2] == p.JOINT_REVOLUTE:
                    p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=max_limit)
                # For prismatic joints, set the joint to its max position
                elif p.getJointInfo(robot_id, joint_index)[2] == p.JOINT_PRISMATIC:
                    p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=max_limit)

        for _ in range(0, 100):
            time.sleep(1/240)
            p.stepSimulation()
        end_effector_position = get_robot_ee_pose(p, robot_id)[0]  # Position of the last link (end effector)

        # Compute the linear distance from the base to the end-effector
        base_position = get_robot_base_pose(p, robot_id)[0] # Get the base position (XYZ)
        print(f"{base_position} {end_effector_position}")
        end_effector_xy = np.array([end_effector_position[0], end_effector_position[1]])  # (x, y) of end effector
        base_xy = np.array([base_position[0], base_position[1]])  # (x, y) of base

        # Compute the Euclidean distance in the x, y plane
        distance_xy = np.linalg.norm(end_effector_xy - base_xy)

        # Now, reset each joint back to its original position
        if reset:
            self.move_arm_joints_to_contracted_position()
        self.max_reachable_distance = distance_xy
        self.max_height = end_effector_position[2]
        return self.max_reachable_distance, self.max_height

    def move_arm_joints_to_contracted_position(self):

        # First contracting the arm.
        # If the arm is larger than min possible then there is no collision
        for arm_index in [*self.link_state_detector.arms_indices, self.link_state_detector.lift_link_index]:
            joint_lower_limit = self.p.getJointInfo(self.robotId, arm_index)[8]

            self.p.setJointMotorControl2(self.robotId, arm_index, 
                                        self.p.POSITION_CONTROL, targetPosition=joint_lower_limit)
            for _ in range(0, 10):
                time.sleep(1/240)
                self.p.stepSimulation()
        return
    

    def contract_arm(self):
        for arm_index in self.link_state_detector.arms_indices:
            joint_lower_limit = self.p.getJointInfo(self.robotId, arm_index)[8]

            self.p.setJointMotorControl2(self.robotId, arm_index, 
                                        self.p.POSITION_CONTROL, targetPosition=joint_lower_limit)
            for _ in range(0, 20):
                time.sleep(1/240)
                self.p.stepSimulation()
        return


    def move_arm_to_max_height(self):

        joint_upper_limit = self.p.getJointInfo(self.robotId, self.link_state_detector.lift_link_index)[9]

        self.p.setJointMotorControl2(self.robotId, self.link_state_detector.lift_link_index, 
                                     self.p.POSITION_CONTROL, targetPosition=joint_upper_limit)
        for _ in range(0, 100):
            time.sleep(1/240)
            self.p.stepSimulation()
        
    