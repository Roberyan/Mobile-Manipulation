
import time

class Grasp:
    def __init__(self, p, robot_id):
        self.p = p
        self.robot_id = robot_id
        self.movable_joints = self.get_movable_joints(robot_id)
        self.simulation_step = 30

    def get_movable_joints(self, robot):
        num_joints = self.p.getNumJoints(robot)
        movable_joints = []
        for i in range(num_joints):
            joint_info = self.p.getJointInfo(robot, i)
            if joint_info[2] != self.p.JOINT_FIXED:
                movable_joints.append((i))
                print("index, name", i, joint_info[1])
        return movable_joints

    

    def set_joint_positions(self, robot, joint_indices, joint_positions, force=None):
    # assert len(joint_positions) == len(movable_joints), "Mismatch in number of positions and movable joints"
        for joint_idx, position in zip(joint_indices, joint_positions):
            # if joint_idx in [1, 2, 3, 4, 5]:
            #     continue
            if not force:
                self.p.setJointMotorControl2(robot, joint_idx, self.p.POSITION_CONTROL, 
                                    targetPosition=position)
            
            if force:
                self.p.setJointMotorControl2(robot, joint_idx, self.p.POSITION_CONTROL, 
                                    targetPosition=position, force=force)
                self.p.stepSimulation()
                time.sleep(1./240.)
            



    def close_gripper(self, robot, left_finger_index, right_finger_index, distance=0.07):
        self.set_joint_positions(robot, [left_finger_index, right_finger_index], 
                            [-distance/2, distance/2])
        
        # Wait for the gripper to close
        for _ in range(self.simulation_step):
            self.p.stepSimulation()
            time.sleep(1./240.)

    def open_gripper(self, robot, left_finger_index, right_finger_index, distance=0.07):
        self.set_joint_positions(robot, [left_finger_index, right_finger_index], 
                            [distance/2, -distance/2])
        
        # Wait for the gripper to open
        for _ in range(self.simulation_step):
            self.p.stepSimulation()
            time.sleep(1./240.)




    def move_arm_to_position(self, robot, target_pos, end_effector_index):
        joint_positions = self.p.calculateInverseKinematics(robot, end_effector_index, target_pos)
        self.set_joint_positions(robot,self.movable_joints, joint_positions)
        
        # Wait for the arm to reach the position
        for _ in range(self.simulation_step):
            self.p.stepSimulation()
            time.sleep(1./240.)


    def grip_and_lift_cup(self, robot, cup_pos, end_effector_index, left_finger_index, right_finger_index):
        # Get cup position

        
        print("moving just above cup")
        # Move arm slightly above the cup
        target_pos = (cup_pos[0]-0.07, cup_pos[1], cup_pos[2] + 0.1)
        self.move_arm_to_position(robot, target_pos, end_effector_index)
        
        print("closing gripper")
        # Close gripper
        self.close_gripper(robot, left_finger_index, right_finger_index)
        
        # Create a constraint to attach the cup to the gripper
        #constraint_id = self.p.createConstraint(robot, end_effector_index, cup_id, -1, self.p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
        

        print("lifting the cup")
        # Lift the cup
        lift_pos = (cup_pos[0]-0.08, cup_pos[1], cup_pos[2] + 0.3)  # Lift 50cm
        
        joint_positions = self.p.calculateInverseKinematics(robot, end_effector_index, lift_pos)
        self.set_joint_positions(robot, self.movable_joints, joint_positions, force=700)
        
        # Wait for the arm to reach the position
        for _ in range(30):
            self.p.stepSimulation()
            time.sleep(1./240.)
        return lift_pos
    
        


    # Grip and lift the cup
# grip_and_lift_cup(robot, cup_pos, end_effector_index, 
#                                         joint_indices['joint_left_finger'], joint_indices['joint_right_finger'])

