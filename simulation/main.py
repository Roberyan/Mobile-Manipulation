import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from utils.tools import *
from utils.grasp import Grasp

# navigation tools
from simulation.astar_global_planner import NavMap
from simulation.astar_navigator import RobotNavigator

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
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

def grasp_init(p):
    grasper = Grasp(p, mobot.robotId)
    constraint = None
    gripped = False
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)
    time.sleep(5)
    end_effector_index = 16
    mug_position =  p.getBasePositionAndOrientation(mug_id)[0]
    drawer_position = p.getBasePositionAndOrientation(drawer_id)[0]
    goal_position = [drawer_position[0]-0.45, drawer_position[1], 0.7]
    base_position = [3.69, 0.07, 3]
    grasper.move_arm_to_position(mobot.robotId, 
                                    target_pos=base_position, 
                                    end_effector_index=end_effector_index)
    return grasper

def keyboard_control():
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


            if (k == ord('u') and (v & p.KEY_WAS_TRIGGERED) and not gripped):
                lift_pos = grasper.grip_and_lift_cup(mobot.robotId, 
                                cup_pos=mug_position,
                                end_effector_index=16,
                                left_finger_index=18,
                                right_finger_index=19)
                gripped=True
                grasper.move_arm_to_position(
                    mobot.robotId,
                    target_pos=[goal_position[0], lift_pos[1], lift_pos[2]],
                    end_effector_index=16
                )
                grasper.move_arm_to_position(
                    mobot.robotId,
                    target_pos=goal_position,
                    end_effector_index=16
                )
                grasper.open_gripper(
                    mobot.robotId,
                    left_finger_index=18,
                    right_finger_index=19
                )

        base_control(mobot, p, forward, turn)
        arm_control(mobot, p, up, stretch, roll, yaw)

        if gripper_open == 1:
            constraint = attach(21, mobot.robotId, 18)
        elif gripper_open == -1:
            detach(constraint)
            constraint = None
        
        mobot.get_observation()

# test for problem area
# p.resetBasePositionAndOrientation(mobot.robotId, (1.534303157375897, -2.8175246336552509, 0.035), [0, 0, 0, 1])

mobot.get_observation()
# keyboard_control()

# global planning based on astar
nav_map = NavMap(p, mobot.robotId, objects_dict, grid_resolution=0.11)
nav_map.label_objects() # capture all objects in the env
# nav_map.show_map() # show 2D astar map
goal_id = objects_dict['drawer'] # goad object
astar_path = nav_map.get_astar_map(mobot.robotId, goal_id) # astar planning from robot's position to goal position
if astar_path is not None:
    # nav_map.visualize_astar(astar_path, mobot.robotId, goal_id) # show 2d astar map with planned path    
    navigator = RobotNavigator(p, mobot, nav_map, astar_path, goal_id) # navigator to move robot
    navigator.show_path_in_world() # show planned path in simulation env
    navigator.move_according_to_path() # move according to planned path
    


total_driving_distance = 0
previous_position, _, _ = get_robot_base_pose(p, mobot.robotId)
current_position = previous_position

constraint = None

navi_flag = False
grasp_flag = False

while (1):
    time.sleep(1./240.)
    keys = p.getKeyboardEvents()


    mobot.get_observation()
    
    current_position, _, _ = get_robot_base_pose(p, mobot.robotId)
    total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
    previous_position = current_position

    if navi_flag == False:
        if current_position[0] > 1.6 and current_position[1] > -0.35:
            print("Reached the goal region! Total driving distance: ", total_driving_distance)
            navi_flag = True
        else:
            print("Total driving distance: ", total_driving_distance)
            print("Current position: ", current_position)
    else:
        print("Reached the goal region! Total driving distance: ", total_driving_distance)
    
    
    if grasp_flag == False:
        mug_position = get_mug_pose(p)
        print("Mug position: ", mug_position)

        if mug_position[0] > 3.3 and mug_position[0] < 3.5 \
            and mug_position[1] > -0.17 and mug_position[1] < 0.25 \
            and mug_position[2] > 0.71 and mug_position[2] < 0.75:
            print("Mug is in the drawer!")
            grasp_flag = True
    else:
        print("Mug is in the drawer!")

    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    print("End-effector position: ", ee_position)
