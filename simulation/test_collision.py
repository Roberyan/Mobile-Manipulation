from main import *

def test_function(p, robot_id):
    import random
    link_s_detector = LinkStateDetector(p, robotId=robot_id)
    collision_checker = CollisionChecker(p, mobot, 
                                         stretched_states=mobot.stretched_joint_states, compressed_states=mobot.compressed_joint_states)
    mobot.move_arm_to_max_height()
    target_arm_pos = get_robot_ee_pose(p, robot_id)[0]
    mobot.move_arm_joints_to_contracted_position()
    current_base_state = get_robot_base_pose(p, robot_id)
    current_base_pos = current_base_state[0]
    current_base_orn = current_base_state[1]
    target_base_pos = (3.13, 0.2171208854914503, 0.0857998984358789)
    relative_angle = calculate_rotation_to_90_counterclockwise(
        p, 
        target_base_pos, 
        target_arm_pos=target_arm_pos,
        current_orientation=[0,0,0,1])
    target_base_orn = get_target_orientation(p, relative_angle, current_base_orn)
    # Bot is simulated to move from current position to new position and orn
    target_link_info = collision_checker.get_link_info_at_target_with_orientation(
        target_base_position=target_base_pos,
        target_base_ori=target_base_orn,
        current_link_info=mobot.compressed_joint_states
    )
    
    # Get the movement volume in the target orientation
    movement_aabbs = collision_checker.get_arm_movement_aabbs(target_arm_pos, target_link_info)
    for link_idx in movement_aabbs:
        visualize_aabb_filled(p, movement_aabbs[link_idx]['aabb'], 
                              color=[random.uniform(0,1), random.uniform(0,1), random.uniform(0,1), 0.3])
        for _ in range(500):
            time.sleep(1/230)
            p.stepSimulation()
        
test_function(p, mobot.robotId)
