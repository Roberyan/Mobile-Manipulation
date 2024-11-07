import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from utils.tools import *

# navigation tools
from simulation.astar_global_planner import NavMap
from simulation.astar_navigator import RobotNavigator

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -9.81)

mobot, objects_dict = init_scene(p)
    
forward=0
turn=0
speed=10
up=0
stretch=0
gripper_open=0
roll=0
yaw=0

mobot.get_observation()

nav_map = NavMap(p, mobot.robotId, objects_dict, grid_resolution=0.11)
nav_map.label_objects()
# nav_map.show_map()
goal_id = objects_dict['cabinet']
astar_path = nav_map.get_astar_map(mobot.robotId, goal_id)

if astar_path is not None:
    # nav_map.visualize_astar(astar_path, mobot.robotId, goal_id)
    
    navigator = RobotNavigator(p, mobot, nav_map, astar_path)
    navigator.show_path_in_world()
    navigator.move_according_to_path()
    
print("--------------------")

def robot_move_according_to_path(nav_map, astar_path):
    # Define movement parameters
    forward_speed = 0.1
    turn_speed = 0.1
    
    def grid_to_world(grid_x, grid_y):
        """
        Convert grid coordinates to world coordinates.
        """
        world_x = nav_map.x_min + grid_x * nav_map.grid_resolution
        world_y = nav_map.y_min + grid_y * nav_map.grid_resolution
        return world_x, world_y
    
    def astar_in_bullet_simulation():
        print("Visualize planned path in the bullet simulation")
        for i in range(len(astar_path) - 1):
            start_point = grid_to_world(astar_path[i][0], astar_path[i][1])
            end_point = grid_to_world(astar_path[i + 1][0],astar_path[i + 1][1])
            start_point = (start_point[0], start_point[1], 0.0)
            end_point = (end_point[0], end_point[1], 0.0)
            p.addUserDebugLine(start_point, end_point, lineColorRGB=[1, 0, 0], lineWidth=5)
            
            # Add a visual sphere at the start point for better visibility
            sphere_radius = nav_map.grid_resolution * 0.5  # Make the size proportional to grid resolution
            p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=sphere_radius,
                rgbaColor=[0, 1, 0, 1],  # Green color
                visualFramePosition=start_point
            )

            # Optionally, add a sphere at the end point (unless it's the last iteration)
            if i == len(astar_path) - 2:
                p.createVisualShape(
                    shapeType=p.GEOM_SPHERE,
                    radius=sphere_radius,
                    rgbaColor=[0, 0, 1, 1],  # Blue color
                    visualFramePosition=end_point
                )
    
    astar_in_bullet_simulation()
    
    def is_within_grid_resolution(current_tuple, aim_tuple):
        return abs(current_tuple[0]-aim_tuple[0])<=nav_map.grid_resolution and \
            abs(current_tuple[1]-aim_tuple[1])<=nav_map.grid_resolution
    
    for grid_x, grid_y in astar_path:
        target_x, target_y = grid_to_world(grid_x, grid_y)

        while True:
            time.sleep(1./240.)
            p.stepSimulation()
            position = p.getLinkState(mobot.robotId, 3)[0]
            current_x, current_y = position[0], position[1]
            if is_within_grid_resolution((current_x, current_y), (target_x, target_y)):
                base_control(mobot, p, forward=0, turn=0)
                break
            
            orientation = p.getLinkState(mobot.robotId, 3)[1]
            _, _, yaw = p.getEulerFromQuaternion(orientation)
            
            # Calculate the direction to the target
            angle_to_target = np.arctan2(target_y - current_y, target_x - current_x)
            angle_diff = angle_to_target - yaw
            
            # Normalize angle_diff to [-pi, pi]
            angle_diff = (angle_diff + np.pi) % (2 * np.pi) - np.pi
            
            if abs(angle_diff) >= 0.1:
                # print(angle_diff)
                turn = np.sign(angle_diff) * turn_speed
                base_control(mobot, p, forward=0, turn=turn)
                continue
            base_control(mobot, p, forward=0, turn=0)
                
            base_control(mobot, p, forward=forward_speed, turn=0)


# constraint = None
# while (1):
#     time.sleep(1./240.)
#     keys = p.getKeyboardEvents()

#     for k,v in keys.items():
#         # moving
#         if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
#             turn = -1
#         if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
#             turn = 0
#         if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
#             turn = 1
#         if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
#             turn = 0
#         if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
#             forward=1
#         if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
#             forward=0
#         if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
#             forward=-1
#         if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
#             forward=0

#         # lifting
#         if (k == ord('z') and (v & p.KEY_WAS_TRIGGERED)):
#             up = 1
#         if (k == ord('z') and (v & p.KEY_WAS_RELEASED)):
#             up = 0
#         if (k == ord('x') and (v & p.KEY_WAS_TRIGGERED)):
#             up = -1
#         if (k == ord('x') and (v & p.KEY_WAS_RELEASED)):
#             up = 0

#         # stretching
#         if (k == ord('a') and (v & p.KEY_WAS_TRIGGERED)):
#             stretch = -1
#         if (k == ord('a') and (v & p.KEY_WAS_RELEASED)):
#             stretch = 0
#         if (k == ord('d') and (v & p.KEY_WAS_TRIGGERED)):
#             stretch = 1
#         if (k == ord('d') and (v & p.KEY_WAS_RELEASED)):
#             stretch = 0

#         # roll
#         if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
#             roll = 1
#         if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
#             roll = 0
#         if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
#             roll = -1
#         if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
#             roll = 0

#         # yaw
#         if (k == ord('y') and (v & p.KEY_WAS_TRIGGERED)):
#             yaw = 1
#         if (k == ord('y') and (v & p.KEY_WAS_RELEASED)):
#             yaw = 0
#         if (k == ord('h') and (v & p.KEY_WAS_TRIGGERED)):
#             yaw = -1
#         if (k == ord('h') and (v & p.KEY_WAS_RELEASED)):
#             yaw = 0


#         # gripper
#         if (k == ord('q') and (v & p.KEY_WAS_TRIGGERED)):
#             gripper_open = -1
#         if (k == ord('q') and (v & p.KEY_WAS_RELEASED)):
#             gripper_open = 0
#         if (k == ord('e') and (v & p.KEY_WAS_TRIGGERED)):
#             gripper_open = 1
#         if (k == ord('e') and (v & p.KEY_WAS_RELEASED)):
#             gripper_open = 0

#     base_control(mobot, p, forward, turn)
#     arm_control(mobot, p, up, stretch, roll, yaw)

#     if gripper_open == 1:
#         constraint = attach(21, mobot.robotId, 18)
#     elif gripper_open == -1:
#         detach(constraint)
#         constraint = None
    
#     mobot.get_observation()


