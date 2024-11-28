from .astar_global_planner import NavMap, AStarNode, Node
from navigation.ee_planner import CollisionChecker
from simulation.stretch import Robot, LinkStateDetector
from utils.tools import get_robot_base_pose
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import heapq
import numpy as np
import matplotlib.pyplot as plt
import heapq
import traceback

class PointPlanner(NavMap):

    def __init__(self, mobot: Robot, p, *args, **kwargs):
        self.mobot = mobot
        self.collision_checker = CollisionChecker(
            p,
            mobot,
            mobot.compressed_joint_states,
            mobot.stretched_joint_states
        )
        self.link_state_detector = LinkStateDetector(
            p, self.mobot.robotId
        )
        self.current_link_info = self.link_state_detector.get_current_link_info()
        super().__init__(p, *args, **kwargs)

    def grid_to_world(self, grid_x, grid_y):
        """
        Convert grid coordinates (grid_x, grid_y) to world coordinates (x, y).
        
        :param grid_x: x-coordinate in grid space
        :param grid_y: y-coordinate in grid space
        :return: tuple (world_x, world_y) of coordinates in world space
        """
        world_x = (grid_x * self.grid_resolution) + self.x_min
        world_y = (grid_y * self.grid_resolution) + self.y_min
        return world_x, world_y

    def get_heuristic(self, node1, node2, goal_id, robot_id, robot_z_range):
        """
        Calculate the heuristic (Euclidean distance) between two nodes.
        """
        
        base_heuristic = np.hypot(node1.x - node2.x, node1.y - node2.y)
        
        # Penalty for proximity to obstacles
        penalty = 0
        proximity_threshold = 2  # Number of grid cells considered 'near' an obstacle
        
        for i in range(-proximity_threshold, proximity_threshold + 1):
            for j in range(-proximity_threshold, proximity_threshold + 1):
                nx, ny = node1.x + i, node1.y + j
                if 0 <= nx < self.grid_size_x and 0 <= ny < self.grid_size_y and self.is_occupied(nx, ny, goal_id, robot_id, robot_z_range):
                    distance_to_obstacle = np.hypot(i, j)
                    node_objects = self.map[nx][ny].get_objects().keys()
                    is_wall = False
                    for obj_name, _ in node_objects:
                        if "wall" in obj_name:
                            is_wall = True
                            break
                    if is_wall:
                        penalty += max(0, (proximity_threshold - distance_to_obstacle)) * 1.5
                    else:        
                        penalty += max(0, proximity_threshold - distance_to_obstacle)  # Apply higher penalty for closer obstacles
        
        return base_heuristic + penalty

    def is_occupied_range(self, x, y, goal_position=None, robot_id=None, robot_z_range=None):

        # Define the four possible configurations to check
        configurations = [
            ((-2,2), (-2, 3)),
            ((-2,2), (-3, 2)),
            ((-2,3), (-2, 2)),
            ((-3,2), (-2, 2))
        ]

        # Assume the grid is free unless all configurations have collisions
        for (x_range, y_range) in configurations:
            not_occupied = True

            # Iterate over the specified range in both x and y directions
            for x_offset in range(x_range[0], x_range[1] + 1):
                for y_offset in range(y_range[0], y_range[1] + 1):
                    check_x = x + x_offset
                    check_y = y + y_offset

                    if self.is_occupied(check_x, check_y, goal_position, robot_id, robot_z_range):
                        not_occupied = False
                        break
                    
            if not_occupied:
                return False

        return True
    
    
    def get_astar_map(self, robot_id, goal_point, consider_radius=True, return_closest=True, visualize=False):
        """
        A* implementation to find a path from robot to a specified goal point.
        
        :param robot_id: ID of the robot.
        :param goal_point: Tuple of (x, y) representing the goal's center point in world coordinates.
        :param consider_radius: Boolean to consider robot's radius for collision checking.
        """
        # # Get robot's center position (from AABB)
        # base_aabb, _ = self.getAABB(robot_id)
        # min_x, min_y, _ = base_aabb[0]
        # max_x, max_y, _ = base_aabb[1]
        robot_center = get_robot_base_pose(self.p, self.mobot.robotId)[0]
        
        # Convert world coordinates to grid coordinates
        start_x, start_y = self.world_to_grid(robot_center[:2])
        goal_x, goal_y = self.world_to_grid(goal_point)
        
        # Create start and goal nodes
        start_node = AStarNode(start_x, start_y, 0.0, -1)
        goal_node = AStarNode(goal_x, goal_y, 0.0, -1)
        
        # Initialize open and closed sets
        open_set = {}
        visited = {}
        open_set[(start_node.x, start_node.y)] = start_node
        
        pq = []
        heapq.heappush(
            pq, 
            (start_node.cost + self.get_heuristic(start_node, goal_node, goal_point, robot_id, self.arm_z_range), (start_node.x, start_node.y))
        )
        closest_node = None
        min_heuristic = float('inf')
        
        while pq:
            _, current_coord = heapq.heappop(pq)
            
            # Skip outdated nodes
            if current_coord not in open_set:
                continue
            
            current = open_set[current_coord]
            node_objects = [obj_tuple for obj_tuple in self.map[current.x][current.y].get_objects().keys()]
            
            visited[current_coord] = current
            current_heuristic = self.get_heuristic(current, goal_node, goal_point, robot_id, self.arm_z_range)
            if current_heuristic < min_heuristic:
                print("current", current.x, current.y, "heuristic", min_heuristic)
                min_heuristic = current_heuristic
                closest_node = current

            # Check if the current node contains the goal point
            if current.x == goal_x and current.y == goal_y:
                print("Path found!")
                return self.reconstruct_path(current, visited)
            
            # Update information
            del open_set[current_coord]
            
            # Explore neighbors (8 grids nearby)
            for action in self.actions:
                new_x = current.x + action[0]
                new_y = current.y + action[1]
                new_cost = current.cost + action[2]  # travel length
                
                # Check if within bounds
                if new_x < 0 or new_x >= self.grid_size_x or new_y < 0 or new_y >= self.grid_size_y:
                    continue
                
                # Skip if already visited
                if (new_x, new_y) in visited:
                    continue
                
                new_node = AStarNode(new_x, new_y, new_cost, (current.x, current.y))

                # Check if available for robot to move
               
                # if consider_radius:
                #     if self.is_occupied_range(new_x, new_y, goal_point, robot_id, self.base_z_range):
                #         continue  
                # else:
                #     if self.is_occupied(new_x, new_y, goal_point, robot_id, self.base_z_range):
                #         continue
                if self.is_occupied(new_x, new_y, goal_point, robot_id, self.base_z_range):
                        continue
                
                # If node is new or has a better path, add it to the open set
                if (new_x, new_y) not in open_set or open_set[(new_x, new_y)].cost > new_node.cost:
                    open_set[(new_x, new_y)] = new_node
                    heapq.heappush(
                        pq, 
                        (new_node.cost + self.get_heuristic(new_node, goal_node, goal_point, robot_id, self.arm_z_range), (new_x, new_y))
                    )
        if return_closest and closest_node:
            print("Closest path found!")
            return self.reconstruct_path(closest_node, visited)
        
        print("No available path found.")
        if visualize:
            self.visualize_astar(None, robot_id, goal_point, visited.keys())
        return None
    
    def collision_detected(self, world_x, world_y, obj_id):
        try:
            current_base_pos = get_robot_base_pose(self.p, self.mobot.robotId)[0]
            target_base_pos = [world_x, world_y, current_base_pos[2]]
            quaternions = [
                [0, 0, 0, 1],         # 0 degrees
                [0, 0, 0.7071, 0.7071],   # 90 degrees
                [0, 0, 1, 0],         # 180 degrees
                [0, 0, -0.7071, 0.7071]   # 270 degrees (-90 degrees)
            ]
            collisions = []
            for orn in quaternions:
                collisions.append(self.collision_checker.check_basic_collision_at_position_orientation_object(
                    target_base_pos,
                    orn, obj_id, current_link_info=self.current_link_info))
        except Exception:
            traceback.print_exc()
            
                
        
        #Even if there is a specific orientation where there is no collision return False
        return all(value is True for value in collisions)

    def is_occupied(self, x, y, goal_point=None, robot_id=None, robot_z_range=None):
        """
        Check if the grid cell at (x, y) is occupied, either universally or for specific conditions.
        
        :param x: X-coordinate of the grid cell.
        :param y: Y-coordinate of the grid cell.
        :param goal_point: Tuple of (x, y) for the goal point, if applicable.
        :param robot_id: ID of the robot.
        :param robot_z_range: Tuple of (min_z, max_z) for the robot's height range.
        :return: True if the cell is occupied, False otherwise.
        """
        if 0 <= x < self.grid_size_x and 0 <= y < self.grid_size_y:
            node = self.map[x][y]
            objects_in_cell = node.get_objects()
            world_x, world_y = self.grid_to_world(x, y)
            # No specific IDs or goal point provided, universal occupation check
            if goal_point is None and robot_id is None:
                return bool(objects_in_cell)
            
            # Compare the robot's z-range with objects' z-ranges in the cell
            robot_min_z, robot_max_z = robot_z_range
            for (obj_name, obj_id), (obj_min_z, obj_max_z) in objects_in_cell.items():
                if obj_id == robot_id:
                    continue
                
                if "wall" in obj_name:
                    return True
                
                if not (robot_max_z <= obj_min_z or robot_min_z >= obj_max_z):
                    return True
                # if self.collision_detected(world_x, world_y, obj_id):
                #     return True
            
            # Check if the goal point coincides with the cell
            if goal_point:
                goal_grid_x, goal_grid_y = self.world_to_grid(goal_point)
                if x == goal_grid_x and y == goal_grid_y:
                    return False  # Goal point itself is not considered an obstacle
        else:
            return True
        
        return False  # No objects or collisions, the cell is free

    def visualize_astar(self, path, robot_id, goal_point, explored_cells=None):
        fig, ax = plt.subplots(figsize=(8, 8))

        # Create a color map for different objects, similar to show_map
        unique_objects = set()
        for i in range(self.grid_size_x):
            for j in range(self.grid_size_y):
                obj_tuples = self.map[i][j].get_objects().keys()
                for obj_tuple in obj_tuples:
                    obj_name, obj_id = obj_tuple
                    if obj_id == robot_id:
                        unique_objects.add(obj_name)

        # Use a color map to assign a unique color to each object
        colormap = plt.get_cmap('tab10')
        colors = {obj_name: colormap(i % 10) for i, obj_name in enumerate(unique_objects)}
        colors['others'] = 'gray'
        colors['astar'] = 'lightblue'
        colors['explored'] = 'lightyellow'

        # Draw grid cells with plt.Rectangle and mark objects using scatter
        for i in range(self.grid_size_x):
            for j in range(self.grid_size_y):
                node = self.map[i][j]
                object_tuples = node.get_objects()
                x = i  # Rectangle grid coordinate
                y = j
                # Add a rectangle for the cell
                ax.add_patch(plt.Rectangle((x, y), 1, 1, color='white', edgecolor='gray', alpha=0.5))

                # If the node has objects, scatter points in the center of the grid cell
                for obj_tuple in object_tuples:
                    obj_name = obj_tuple[0]
                    if obj_name not in colors.keys():
                        obj_name = "others"
                    ax.scatter(x + 0.5, y + 0.5, color=colors[obj_name], s=50)

        if explored_cells is not None:
            # Mark all explored cells
            for (x, y) in explored_cells:
                ax.add_patch(plt.Rectangle((x, y), 1, 1, color=colors['explored'], alpha=0.5))

        # Draw the A* path (if found)
        if path:
            path_x, path_y = zip(*path)
            ax.plot([x + 0.5 for x in path_x], [y + 0.5 for y in path_y], color='blue', linewidth=2, label='A* Path')

            check_radius = 3
            radius_cells = set()
            for (center_x, center_y) in path:
                x_range = range(-check_radius, check_radius + 1)
                y_range = range(-check_radius, check_radius + 1)

                # Check each cell within the radius
                for x_offset in x_range:
                    for y_offset in y_range:
                        check_x = center_x + x_offset
                        check_y = center_y + y_offset

                        # Ensure the cell is within grid bounds
                        if not self.is_occupied(check_x, check_y, goal_point=goal_point, robot_id=robot_id, robot_z_range=self.base_z_range):
                            radius_cells.add((check_x, check_y))

            for (x, y) in radius_cells:
                ax.add_patch(plt.Rectangle((x, y), 1, 1, color=colors['astar'], alpha=0.3))

        # Mark the robot position (green circle)
        base_aabb, _ = self.getAABB(robot_id)
        robot_center = (
            (base_aabb[0][0] + base_aabb[1][0]) / 2,
            (base_aabb[0][1] + base_aabb[1][1]) / 2
        )
        robot_x, robot_y = self.world_to_grid(robot_center)
        ax.scatter(robot_x + 0.5, robot_y + 0.5, color='green', s=100, label='Robot', marker='o')
        self.show_direction(ax, robot_x, robot_y)

        # Mark the goal position (red star)
        goal_x, goal_y = self.world_to_grid(goal_point)
        ax.scatter(goal_x + 0.5, goal_y + 0.5, color='red', s=200, label='Goal', marker='*')

        # Set grid axis labels to match grid coordinates
        ax.set_xlim([0, self.grid_size_x])
        ax.set_ylim([0, self.grid_size_y])
        ax.set_xticks(np.arange(0, self.grid_size_x + 1, 1))
        ax.set_yticks(np.arange(0, self.grid_size_y + 1, 1))
        ax.set_xticklabels(np.arange(0, self.grid_size_x + 1, 1))
        ax.set_yticklabels(np.arange(0, self.grid_size_y + 1, 1))

        # Set axis labels and title
        ax.set_xlabel('Grid X')
        ax.set_ylabel('Grid Y')
        ax.set_title('A* Path Visualization')
        ax.grid(True)

        ax.legend(loc='upper right', bbox_to_anchor=(1.2, 1), title="Legend")
        plt.show()
