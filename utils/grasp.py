
import time
from utils.tools import *
from navigation.collision_detecting_navigator import CollisionDetectingNavigator
from navigation.point_planner import PointPlanner
from navigation.ee_planner import RobotEndEffectorPlanner, CollisionChecker
from simulation.stretch import Robot


class Grasp:
    def __init__(self, p, objects_dict, mobot: Robot):
        self.p = p
        self.robot_id = mobot.robotId
        self.movable_joints = mobot.get_movable_joints()
        self.simulation_step = 30
        self.mobot = mobot
        self.nav_map = PointPlanner(mobot, self.p, self.mobot.robotId, objects_dict, grid_resolution=0.15)
        self.objects_dict = objects_dict
        self.collision_checker = CollisionChecker(self.p, mobot, 
                                                  mobot.compressed_joint_states, 
                                                  mobot.stretched_joint_states)
        
    

    def set_joint_positions(self, joint_indices, joint_positions, force=None, move_wheels=False):
    # assert len(joint_positions) == len(movable_joints), "Mismatch in number of positions and movable joints"
        for joint_idx, position in zip(joint_indices, joint_positions):
            if joint_idx in [1, 2, 3, 4, 5] and not move_wheels:
                continue
            if not force:
                self.p.setJointMotorControl2(self.robot_id, joint_idx, self.p.POSITION_CONTROL, 
                                    targetPosition=position)
            
            if force:
                self.p.setJointMotorControl2(self.robot_id, joint_idx, self.p.POSITION_CONTROL, 
                                    targetPosition=position, force=force)
                self.p.stepSimulation()
                time.sleep(1./240.)
            



    def close_gripper(self, left_finger_index, right_finger_index, distance=0.07):
        self.set_joint_positions( [left_finger_index, right_finger_index], 
                            [-distance/2, distance/2])
        
        # Wait for the gripper to close
        for _ in range(self.simulation_step):
            self.p.stepSimulation()
            time.sleep(1./240.)

    def open_gripper(self, left_finger_index, right_finger_index, distance=0.07):
        self.set_joint_positions( [left_finger_index, right_finger_index], 
                            [distance/2, -distance/2])
        
        # Wait for the gripper to open
        for _ in range(self.simulation_step):
            self.p.stepSimulation()
            time.sleep(1./240.)




    def move_arm_to_position(self, target_pos, end_effector_index, move_wheels=False):
        joint_positions = self.p.calculateInverseKinematics(self.robot_id, end_effector_index, target_pos)
        self.set_joint_positions(self.movable_joints, joint_positions, move_wheels)
        
        # Wait for the arm to reach the position
        for _ in range(self.simulation_step):
            self.p.stepSimulation()
            time.sleep(1./240.)

    def move_to_closest_point(self, goal_position):
        self.nav_map.label_objects() # capture all objects in the env
        # nav_map.show_map() # show 2D astar map
        goal_position = [goal_position[0], goal_position[1]] # mug object
        astar_path = self.nav_map.get_astar_map(self.mobot.robotId, goal_point=goal_position) # astar planning from robot's position to goal position
        if astar_path is not None:
            # nav_map.visualize_astar(astar_path, mobot.robotId, goal_id) # show 2d astar map with planned path    
            navigator = CollisionDetectingNavigator(self.p, self.mobot, self.nav_map, astar_path, None) # navigator to move robot
            navigator.show_path_in_world() # show planned path in simulation env
            navigator.move_according_to_path() # move according to planned path

    def get_collision_free_base_points_near_target(self, target_object_id, target_pos):
        """
            Returns all points where the bot can reach object without collision
            all points are sorted with the nearest point to current base pose
        """
        planner = RobotEndEffectorPlanner(
            self.p, self.mobot, self.objects_dict, 
            movable_joints=self.mobot.get_movable_joints(),
            target_object_id=target_object_id,
            max_reachable_distance=self.mobot.max_reachable_distance, 
            max_height=self.mobot.max_height)
        collision_free_new_positions = planner.plan_end_effector_path(
                target_pos   
            )
        return collision_free_new_positions
    


        
    def move_to_nearest_collision_free_point(self, collision_free_new_positions):
        """
            There are two possible collisions before picking an object
            1. The arm collides while stretching, so we need to find a point where base and arm doesnt collide
            2. To reach the point there might be obstacles so find collision free path to the point
            3. This function tries to find a path to point where arm can reach and pick object

        """
        if collision_free_new_positions is not None:
            for collision_free_new_position in collision_free_new_positions:
                goal_position = collision_free_new_position[0], collision_free_new_position[1]
                self.nav_map.label_objects() # capture all objects in the env
                # nav_map.show_map() # show 2D astar map
                astar_path_pickup = self.nav_map.get_astar_map(
                    self.mobot.robotId, goal_point=goal_position, return_closest=False) # astar planning from robot's position to goal position
                if astar_path_pickup is not None:
                    break
            
            if astar_path_pickup is not None:
                # nav_map.visualize_astar(astar_path, mobot.robotId, goal_id) # show 2d astar map with planned path    
                navigator = CollisionDetectingNavigator(self.p, self.mobot, self.nav_map, astar_path_pickup, None) # navigator to move robot
                navigator.show_path_in_world() # show planned path in simulation env
                navigator.move_according_to_path() # move according to planned path


    def lift_object(self, object_id=None):
        
        target_pos = self.p.getBasePositionAndOrientation(object_id)[0]
        print(f"Target pos {target_pos}")

        # Move to a point closest to the target object
        self.move_to_closest_point(goal_position=target_pos)

        # Find closest collision free points near target object
        collision_free_points = self.get_collision_free_base_points_near_target(object_id, target_pos)

        # Move to closest point
        self.move_to_nearest_collision_free_point(collision_free_points)

        # Move the arm to the top
        # Collision for this is already checked
        self.mobot.move_arm_to_max_height()


        current_base_pos = get_robot_base_pose(self.p, self.robot_id)

        smoothly_rotate_arm_to_position(self.p, self.mobot.robotId, 
                                        3, 
                            base_position=current_base_pos, 
                            target_position=target_pos )

        
        
        target_pos = (target_pos[0]-0.07, target_pos[1], target_pos[2] + 0.1)
        print(f"to move pos {target_pos}")
        self.move_arm_to_position(target_pos, self.mobot.end_effector_idx)
        
        print("Closing gripper")
        # Close gripper
        self.close_gripper(self.mobot.left_finger_index, self.mobot.right_finger_index)
        
        
        attach(self.p, object_id, self.robot_id, self.mobot.end_effector_idx)

        print("lifting the object")
        
        # lift_pos = (target_pos[0], target_pos[1], current_pos[2])  # Lift 50cm
        
        # joint_positions = self.p.calculateInverseKinematics(
        #     self.robot_id, self.mobot.end_effector_idx, lift_pos)
        # self.set_joint_positions(self.movable_joints, joint_positions, force=700)
        
        self.mobot.move_arm_to_max_height()
        
        # Wait for the arm to reach the position
        for _ in range(30):
            self.p.stepSimulation()
            time.sleep(1./240.)
        
    
        