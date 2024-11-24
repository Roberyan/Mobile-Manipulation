import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from utils.tools import *
from utils.grasp import Grasp
#from inference import grasp_generator
# navigation tools
from navigation.astar_global_planner import NavMap
from navigation.astar_navigator import RobotNavigator
from navigation.point_planner import PointPlanner
from navigation.ee_planner import RobotEndEffectorPlanner, CollisionChecker





p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setGravity(0, 0, -9.81)


mobot, objects_dict, mug_id, drawer_id = init_scene(p)
    
forward=0
turn=0
speed=10
up=0
stretch=0
gripper_open=0
roll=0
yaw=0


def keyboard_control():
    forward=0
    turn=0
    speed=10
    up=0
    stretch=0
    gripper_open=0
    roll=0
    yaw=0
    mug_position =  p.getBasePositionAndOrientation(mug_id)[0]
    drawer_position = p.getBasePositionAndOrientation(drawer_id)[0]
    goal_position = [drawer_position[0]-0.45, drawer_position[1], 0.7]
    constraint = None
    grasper = grasp_init(p)
    while (1):
        time.sleep(1./240.)
        keys = p.getKeyboardEvents()

        for k,v in keys.items():
            # moving
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                turn = -1
            if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
                turn = 0
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                turn = 1
            if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
                turn = 0
            if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                forward=1
            if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
                forward=0
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
                forward=-1
            if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
                forward=0

            # lifting
            if (k == ord('z') and (v & p.KEY_WAS_TRIGGERED)):
                up = 1
            if (k == ord('z') and (v & p.KEY_WAS_RELEASED)):
                up = 0
            if (k == ord('x') and (v & p.KEY_WAS_TRIGGERED)):
                up = -1
            if (k == ord('x') and (v & p.KEY_WAS_RELEASED)):
                up = 0

            # stretching
            if (k == ord('a') and (v & p.KEY_WAS_TRIGGERED)):
                stretch = -1
            if (k == ord('a') and (v & p.KEY_WAS_RELEASED)):
                stretch = 0
            if (k == ord('d') and (v & p.KEY_WAS_TRIGGERED)):
                stretch = 1
            if (k == ord('d') and (v & p.KEY_WAS_RELEASED)):
                stretch = 0

            # roll
            if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
                roll = 1
            if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
                roll = 0
            if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
                roll = -1
            if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
                roll = 0

            # yaw
            if (k == ord('y') and (v & p.KEY_WAS_TRIGGERED)):
                yaw = 1
            if (k == ord('y') and (v & p.KEY_WAS_RELEASED)):
                yaw = 0
            if (k == ord('h') and (v & p.KEY_WAS_TRIGGERED)):
                yaw = -1
            if (k == ord('h') and (v & p.KEY_WAS_RELEASED)):
                yaw = 0


            # gripper
            if (k == ord('q') and (v & p.KEY_WAS_TRIGGERED)):
                gripper_open = -1
            if (k == ord('q') and (v & p.KEY_WAS_RELEASED)):
                gripper_open = 0
            if (k == ord('e') and (v & p.KEY_WAS_TRIGGERED)):
                gripper_open = 1
            if (k == ord('e') and (v & p.KEY_WAS_RELEASED)):
                gripper_open = 0


        base_control(mobot, p, forward, turn)
        arm_control(mobot, p, up, stretch, roll, yaw)

        if gripper_open == 1:
            constraint = attach(21, mobot.robotId, 18)
        elif gripper_open == -1:
            detach(constraint)
            constraint = None
        
        mobot.get_observation()




# test for problem area
# (3.1327990508778294, -0.4171208854914503, 0.0857998984358789) - end state near drawer
#p.resetBasePositionAndOrientation(mobot.robotId, (1.534303157375897, -2.8175246336552509, 0.035), [0, 0, 0, 1])
#p.resetBasePositionAndOrientation(mobot.robotId, (3.1327990508778294, -0.4171208854914503, 0.0857998984358789), [0, 0, 0, 1])

#p.resetBasePositionAndOrientation(mobot.robotId, 
                                #   (3.563093921139152, 0.7550381406393756, 0.1415997968717578), 
                                #   [0, 0, 0.7071, 0.7071])


total_driving_distance = 0
previous_position, _, _ = get_robot_base_pose(p, mobot.robotId)
current_position = previous_position


constraint = None

navi_flag = False
grasp_flag = False
gripped = False
grasper = Grasp(p, objects_dict, mobot)

while (1):
    
    time.sleep(1./240.)
    keys = p.getKeyboardEvents()


    mobot.get_observation()
    
    current_position, _, _ = get_robot_base_pose(p, mobot.robotId)
    total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
    previous_position = current_position

    # if navi_flag == False:
    #     if current_position[0] > 1.6 and current_position[1] > -0.35:
    #         print("Reached the goal region! Total driving distance: ", total_driving_distance)
    #         navi_flag = True
    #     else:
    #         print("Total driving distance: ", total_driving_distance)
    #         print("Current position: ", current_position)
    # else:
    #     print("Reached the goal region! Total driving distance: ", total_driving_distance)
    
    
    # if grasp_flag == False:
    #     mug_position = get_mug_pose(p)
    #     print("Mug position: ", mug_position)

    #     if mug_position[0] > 3.3 and mug_position[0] < 3.5 \
    #         and mug_position[1] > -0.17 and mug_position[1] < 0.25 \
    #         and mug_position[2] > 0.71 and mug_position[2] < 0.75:
    #         print("Mug is in the drawer!")
    #         grasp_flag = True
    # else:
    #     print("Mug is in the drawer!")

    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    print("End-effector position: ", ee_position)
    for k,v in keys.items():
        if (k == ord('u') and (v & p.KEY_WAS_TRIGGERED) and not gripped):
            # grasp_gen = grasp_generator.GraspGenerator(
            #     saved_model_path='/Users/savithasuresh/NUS/lectures/robotics/project/Mobile-Manipulation/trained-models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93',
            #     cam_id=mobot.camera_index)
            # cup_position, grasp_pose, grasp_angle = grasp_gen.generate()
            # print("Grasp model cup_position", cup_position)
            lift_pos = grasper.lift_object(object_id=21)
            gripped=True
            

    # grasper = Grasp(p, mobot.robotId)
    # grasper.move_arm_to_position(
    #                 mobot.robotId,
    #                 target_pos=[0.27, -0.71, 0.92],
    #                 end_effector_index=18
    #             )
    # motion_planning_test(p, mobot.robotId, [0.27, -0.71, 0.92])
    # time.sleep(2)
    # grasper.move_arm_to_position(
    #                 mobot.robotId,
    #                 target_pos=[-1.70, -3.70, 0.46] ,
    #                 end_effector_index=18
    #             )
    # motion_planning_test(p, mobot.robotId, [-1.70, -3.70, 0.46])
    # time.sleep(2)
    # grasper.move_arm_to_position(
    #                 mobot.robotId,
    #                 target_pos=[1.45, -1.68, 0.59],
    #                 end_effector_index=18
    #             )
    # motion_planning_test(p, mobot.robotId, [1.45, -1.68, 0.59])
    # time.sleep(2)

