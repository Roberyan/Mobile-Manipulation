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
target_orn = p.getQuaternionFromEuler([0, 0, np.pi/4])
# p.resetBasePositionAndOrientation(mobot.robotId, (1.33, -2.8175246336552509, 0.035), [0,0,0.7071,-0.7071])
p.resetDebugVisualizerCamera(
    cameraDistance=7.32,  # Camera distance from target
    cameraYaw=68.96,       # Camera yaw (rotation around the vertical axis)
    cameraPitch=-69.06,    # Camera pitch (rotation around the horizontal axis)
    cameraTargetPosition=[0, 0, 0]  # Camera target position (where it's looking)
)
p.resetBasePositionAndOrientation(mobot.robotId,(-0.9142153472756134, -4.193985614188303, 0.37403600000000004), (0.0, 0.0, -0.23054305375680756, 0.9730621256448561))
#p.resetBasePositionAndOrientation(mobot.robotId, (3.3696472605102656, 0.5533268735036863, 0.28802700000000003), (-0.0, -0.0, 0.13132448905385274, 0.9913394366082409))

p.resetBasePositionAndOrientation(18, (-1.0000000000000009, -4.619999999999997, 0.1876708836682638), [0, 1, 0, 1])
#p.resetBasePositionAndOrientation(17, (-1.2, -4.619999999999997, 0.1876708836682638), [0, 1, 0, 1])




#resetBasePositionAndOrientation(mobot.robotId, (3.1327990508778294, -0.4171208854914503, 0.0857998984358789), [0, 0, 0, 1])

# p.resetBasePositionAndOrientation(mobot.robotId, 
                                #   (3.663093921139152, 0.5550381406393756, 0.1415997968717578), 
                                #   [0, 0, 1, 0])

# p.resetBasePositionAndOrientation(mobot.robotId, 
#                                   (3.4, -2.7, 0.19089074842113535), 
#                                   [0, 0, 1, 0])


total_driving_distance = 0
previous_position, _, _ = get_robot_base_pose(p, mobot.robotId)
current_position = previous_position


constraint = None

navi_flag = False
grasp_flag = False
gripped = False
mobot.obj_aabbs = get_all_obj_aabb(p)
grasper = Grasp(p, objects_dict, mobot)

while (1):
    
    time.sleep(1./240.)
    keys = p.getKeyboardEvents()


    mobot.get_observation()
    
    current_position, _, _ = get_robot_base_pose(p, mobot.robotId)
    total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
    previous_position = current_position

    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    print("End-effector position: ", ee_position)
    for k,v in keys.items():
        if (k == ord('u') and (v & p.KEY_WAS_TRIGGERED) and not gripped):
            # grasp_gen = grasp_generator.GraspGenerator(
            #     saved_model_path='/Users/savithasuresh/NUS/lectures/robotics/project/Mobile-Manipulation/trained-models/jacquard-rgbd-grconvnet3-drop0-ch32/epoch_48_iou_0.93',
            #     cam_id=mobot.camera_index)
            # cup_position, grasp_pose, grasp_angle = grasp_gen.generate()
            # print("Grasp model cup_position", cup_position)
            #grasper.lift_object(object_id=21)
            
            # # # p.resetBasePositionAndOrientation(21, current_base, [0,0,0,1])
            # p.resetBasePositionAndOrientation(18, current_base, [0,0,0,1])
            # # # # grasper.attach_obj_to_link(21)
            # grasper.attach_obj_to_link(18, link_id=0)
            # gripped=True

            obj_id = 17
            grasper.collect_object(object_id=obj_id)
            # print("base_orn", get_robot_base_pose(p, mobot.robotId))
            current_base = get_robot_base_pose(p, mobot.robotId)[0]

        #Attaching object to base, resetting to not affect movement
            # p.resetBasePositionAndOrientation(obj_id, current_base, [0, 0, 0, 1])

            # p.stepSimulation()
            # time.sleep(1/240)
            # grasper.attach_obj_to_link(obj_id, 0)
            # time.sleep(5)
            # print("pose after pickup", get_robot_base_pose(p, mobot.robotId))
            grasper.drop_object_at_goal(goal_obj_id=19)
            print("base_orn after drop", get_robot_base_pose(p, mobot.robotId))
            

    # grasper = Grasp(p, mobot.robotId)x
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

