
from navigation.astar_navigator import RobotNavigator
from navigation.ee_planner import CollisionChecker
from utils.tools import *
from simulation.stretch import base_control
import traceback

class CollisionDetectingNavigator(RobotNavigator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.collision_checker = CollisionChecker(
            self.p,
            self.robot,
            self.robot.compressed_joint_states,
            self.robot.stretched_joint_states
        )

    def visualize_point(self, pt):
       
        if len(pt) == 2:
            x, y = pt
            return self.p.createMultiBody(
                baseVisualShapeIndex=self.random_pt_id,
                basePosition=(x, y, 0)  # Adjust z-axis for better visibility
            )
        elif len(pt) == 3:
            return self.p.createMultiBody(
                baseVisualShapeIndex=self.random_pt_id,
                basePosition=pt
                )
    
    def get_current_base_position_2D(self):
        position = get_robot_base_pose(self.p, self.robot.robotId)[0]
        return position[0], position[1]
        

    def find_possible_better_alternative(self, aim_pos, current_orn, current_pos):
        aim_tuple = (aim_pos[0], aim_pos[1])
        if len(self.world_path) > 3:
            aim_points = self.world_path[:3]
            new_points = find_perpendicular_line_points(aim_points)
        else:
            new_points = generate_surrounding_points(aim_tuple)
        
        final_pt = None
        for pt in new_points:
            try:
                target_pt = [pt[0], pt[1], current_pos[2]]
                v_pt = self.visualize_point(pt)
                current_collide = self.check_current_collision(target_pt, current_orn, current_pos)
                if not current_collide:
                    print("new points no current collide")
                    final_pt = pt
                    break
                reverse_collide = self.check_reverse_collision(target_pt, current_orn, current_pos)
                if not reverse_collide:
                    print("new points no reverse collide")
                    self.change_mode()
                    final_pt =  pt
                    break
            except Exception:
                traceback.print_exc()
            finally:
                if v_pt:
                    self.p.removeBody(v_pt)
        if v_pt:
            self.p.removeBody(v_pt)
        return final_pt
    
    def get_target_orn_for_pt(self, aim_pos, current_orn, current_pos):
        if self.reverse_move % 2 !=0:
            reverse=True
        else:
            reverse=False
        target_angle = calculate_rotation_angle(self.p, current_pos, aim_pos, current_orn, reverse=reverse )
        target_base_orn = get_target_orientation(self.p, target_angle, current_orn)
        return target_base_orn
        
    def check_reverse_collision(self, aim_pos, current_orn, current_pos):
        target_same_orn = self.get_target_orn_for_pt(aim_pos, current_orn, current_pos)
        target_reverse_orn = get_reverse_quaternion_pybullet(self.p, current_pos, target_same_orn)
        return self.collision_checker.check_basic_collision_at_position_orientation(
            aim_pos, target_reverse_orn)
    
    def check_current_collision(self, aim_pos, current_orn, current_pos):
        target_orn = self.get_target_orn_for_pt(aim_pos, current_orn, current_pos)
        return self.collision_checker.check_basic_collision_at_position_orientation(
            aim_pos, target_orn)

    def if_change_mode_allowed(self):
        current_pos_state = get_robot_base_pose(self.p, self.robot.robotId)
        current_pos = current_pos_state[0]
        current_orn = current_pos_state[1]
        
        reverse_collision = self.collision_checker.check_basic_collision_at_position_orientation(current_pos, current_orn)
        if reverse_collision:
            return False
        current_collision = self.collision_checker.check_basic_collision_at_position(current_pos)
        if not current_collision:
            return True
        
    def if_within_range(self, aim_tuple, range=1):
        # special treat for robot base aabb measurement
        bot_base_pose = get_robot_base_pose(self.p, self.robot.robotId)[0]

        return np.linalg.norm(np.array(bot_base_pose[:2]) - np.array(aim_tuple)) <= range
        
    def move_according_to_path(self):
        self.move_arm_to_base()
        while self.world_path:
            aim_tuple = self.mode_decide(self.world_path[0])
            if (aim_tuple is None):
                break
            aim_x, aim_y = aim_tuple
            self.exploring_id = self.visualize_sampled_points((aim_x, aim_y))
            print("reverse_mode", self.reverse_move)
            # real action part
            while True:
                try:
                    time.sleep(1. / 240.)
                    self.p.stepSimulation()
                    
                    if self.if_within_range((aim_x, aim_y), 0.1):
                        base_control(self.robot, self.p, forward=0, turn=0)
                        break
                    
                    # Turn toward the target direction
                    self.turn_to_position(aim_x, aim_y)
                    
                    # if self.if_close_enough((current_x, current_y), (aim_x, aim_y)):
                    if self.if_within_range((aim_x, aim_y), 0.1):
                        base_control(self.robot, self.p, forward=0, turn=0)
                        break
                    
                    #print("Moving...")
                    base_control(self.robot, self.p, forward=self.forward_speed, turn=0)
                    self.freeze_arm()
                except Exception:
                    traceback.print_exc()
                    pass
                
            self.remove_sampled_points(self.exploring_id)
            self.world_path.pop(0)
            self.p.removeBody(self.nav_path_visualize_ids.pop(0))

        while(self.world_path):
            self.remove_sampled_points(self.exploring_id)
            self.world_path.pop(0)
            self.p.removeBody(self.nav_path_visualize_ids.pop(0))

        
        print("Goal object should be nearby.")


    # decide move forward or reverse to go, currently dummy judgement
    def mode_decide(self, aim_tuple, zoom=1.5):
        
        #change_flag = self.if_change_mode_allowed()
        current_pos_state = get_robot_base_pose(self.p, self.robot.robotId)
        current_base_pos = current_pos_state[0]
        current_orn = current_pos_state[1]
        aim_pos = [aim_tuple[0], aim_tuple[1], current_base_pos[2]]

        current_collide = self.check_current_collision(aim_pos, current_orn, current_base_pos)
        
        if not current_collide:
            print("No current collision")
            # if forward and reverse move is the same dont change, rotation is unnecessary
            return aim_tuple
        reverse_collide = self.check_reverse_collision(aim_pos, current_orn, current_base_pos)
        
        if not reverse_collide:
            print("No reverse collision")
            # current mode is not the optimized, change mode
            self.change_mode()
            return aim_tuple

        if current_collide and reverse_collide:
            aim_tuple = self.find_possible_better_alternative(aim_pos, current_orn, current_base_pos)
            
        return aim_tuple
    
