import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from utils.tools import *
from utils.grasp import Grasp
from navigation.astar_global_planner import NavMap
from navigation.astar_navigator import RobotNavigator
from navigation.point_planner import PointPlanner
from navigation.ee_planner import RobotEndEffectorPlanner, CollisionChecker





p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setGravity(0, 0, -9.81)


mobot, objects_dict, mug_id, drawer_id = init_scene(p)
p.resetDebugVisualizerCamera(
    cameraDistance=7.32,  # Camera distance from target
    cameraYaw=68.96,       # Camera yaw (rotation around the vertical axis)
    cameraPitch=-69.06,    # Camera pitch (rotation around the horizontal axis)
    cameraTargetPosition=[0, 0, 0]  # Camera target position (where it's looking)
)

gripped = False
mobot.obj_aabbs = get_all_obj_aabb(p)
grasper = Grasp(p, objects_dict, mobot)

while (1):
    
    time.sleep(1./240.)
    keys = p.getKeyboardEvents()
    print("Press 'u' to start")
    for k,v in keys.items():
        if (k == ord('u') and (v & p.KEY_WAS_TRIGGERED) and not gripped):
            obj_id = 18
            grasper.collect_object(object_id=obj_id)
            grasper.drop_object_at_goal(goal_obj_id=19)
            gripped=True
            

