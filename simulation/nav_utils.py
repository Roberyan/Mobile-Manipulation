import numpy as np
import matplotlib.pyplot as plt
import heapq

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.objects = {} # store object and its z coordinate
    
    def add_object(self, object_id, z_range):
        self.objects[object_id] = z_range
    
    def remove_object(self, object_id):
        if object_id in self.objects:
            del self.objects[object_id]
    
    def get_objects(self):
        return self.objects

class AStarNode:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

class NavMap:
    # for moving actions in the map
    actions = [
        [1, 0, 1],  # Right
        [0, 1, 1],  # Up
        [-1, 0, 1],  # Left
        [0, -1, 1],  # Down
        [1, 1, np.sqrt(2)],  # Diagonal up-right
        [1, -1, np.sqrt(2)],  # Diagonal down-right
        [-1, 1, np.sqrt(2)],  # Diagonal up-left
        [-1, -1, np.sqrt(2)]  # Diagonal down-left
    ]
    
    def __init__(self, p, plane_id, grid_resolution,) -> None:
        self.p = p
        self.grid_resolution = grid_resolution
        # Initialize min and max bounds with extreme values
        self.x_min, self.y_min = float('inf'), float('inf')
        self.x_max, self.y_max = float('-inf'), float('-inf')
        
        num_bodies = p.getNumBodies()
        object_ids = [p.getBodyUniqueId(i) for i in range(num_bodies)]
        for obj_id in object_ids:
            if obj_id == plane_id:
                continue
            
            obj_aabb = self.getAABB(obj_id)
            min_x, min_y, _ = obj_aabb[0]
            max_x, max_y, _ = obj_aabb[1]

            # Update the min and max coordinates
            self.x_min = min(self.x_min, min_x)
            self.y_min = min(self.y_min, min_y)
            self.x_max = max(self.x_max, max_x)
            self.y_max = max(self.y_max, max_y)
                
        self.grid_size_x = int((self.x_max-self.x_min)/grid_resolution)
        self.grid_size_y = int((self.y_max-self.y_min)/grid_resolution)
        self.map = [[Node(x, y) for y in range(self.grid_size_y)] for x in range(self.grid_size_x)]
        
        self.background_id = plane_id
           
    def label_boundary(self):
        # label boundary
        for y in range(self.grid_size_y):
            self.map[0][y].add_object(self.background_id, 0)
            self.map[self.grid_size_x-1][y].add_object(self.background_id, 0)
        
        for x in range(self.grid_size_x):
            self.map[x][0].add_object(self.background_id, 0)
            self.map[x][self.grid_size_y-1].add_object(self.background_id, 0)
    
    def label_objects(self, no_label_id=[]):
        no_label_id.append(self.background_id)
        num_bodies = self.p.getNumBodies()
        object_ids = [self.p.getBodyUniqueId(i) for i in range(num_bodies)]
        for obj_id in object_ids:
            if obj_id in no_label_id:
                continue
            self.add_object(obj_id)
    
    def add_object(self, object_id):
        obj_aabb = self.getAABB(object_id)
        min_x, min_y, min_z = obj_aabb[0]
        max_x, max_y, max_z = obj_aabb[1]
        
        grid_min_x = int((min_x - self.x_min)/self.grid_resolution)
        grid_max_x = int((max_x - self.x_min)/self.grid_resolution)
        grid_min_y = int((min_y - self.y_min)/self.grid_resolution)
        grid_max_y = int((max_y - self.y_min)/self.grid_resolution)
        
        # Clamp the indices to be within valid bounds
        grid_min_x = max(0, min(grid_min_x, self.grid_size_x - 1))
        grid_max_x = max(0, min(grid_max_x, self.grid_size_x - 1))
        grid_min_y = max(0, min(grid_min_y, self.grid_size_y - 1))
        grid_max_y = max(0, min(grid_max_y, self.grid_size_y - 1))
        
        for i in range(grid_min_x, grid_max_x+1):
            for j in range(grid_min_y, grid_max_y+1):
                self.map[i][j].add_object(object_id, (min_z,max_z))
    
    def show_map(self):
        fig, ax = plt.subplots(figsize=(8, 8))

        # Create a color map for different objects
        unique_objects = set()
        for i in range(self.grid_size_x):
            for j in range(self.grid_size_y):
                unique_objects.update(self.map[i][j].get_objects().keys())

        # Use a color map to assign a unique color to each object
        colormap = plt.get_cmap('tab20')
        colors = {obj_id: colormap(i % 20) for i, obj_id in enumerate(unique_objects)}

        # Draw grid cells with plt.Rectangle and mark objects using scatter
        for i in range(self.grid_size_x):
            for j in range(self.grid_size_y):
                node = self.map[i][j]
                objects = node.get_objects()
                x = i  # Rectangle grid coordinate
                y = j
                # Add a rectangle for the cell
                ax.add_patch(plt.Rectangle((x, y), 1, 1, color='lightgray', edgecolor='gray', alpha=0.5))

                # If the node has objects, scatter points in the center of the grid cell
                for obj_id in objects:
                    ax.scatter(x + 0.5, y + 0.5, color=colors[obj_id], s=50)

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
        ax.set_title('2D Grid Map with Objects')
        ax.grid(True)

        # Create a custom legend to show the object ID with corresponding colors
        handles = [plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=colors[obj_id], markersize=10, label=f'Object {obj_id}') for obj_id in unique_objects]
        ax.legend(handles=handles, loc='upper right', bbox_to_anchor=(1.2, 1), title="Object IDs")

        plt.gca().set_aspect('equal', adjustable='box')
        plt.tight_layout()
        plt.show()

    # Provided getAABB fix the problem
    def getLinkInfo(self, object_id):
        numJoint = self.p.getNumJoints(object_id)
        LinkList = ['base']
        for jointIndex in range(numJoint):
            jointInfo = self.p.getJointInfo(object_id, jointIndex)
            link_name = jointInfo[12]
            if link_name not in LinkList:
                LinkList.append(link_name)
        return LinkList

    def getNumLinks(self, object_id):
        return len(self.getLinkInfo(object_id))
    
    def getAABB(self, object_id):
        numLinks = self.getNumLinks(object_id)
        AABB_List = []
        for link_id in range(-1, numLinks - 1):
            AABB_List.append(self.p.getAABB(object_id, link_id))
        AABB_array = np.array(AABB_List)
        AABB_obj_min = np.min(AABB_array[:, 0, :], axis=0)
        AABB_obj_max = np.max(AABB_array[:, 1, :], axis=0)
        AABB_obj = np.array([AABB_obj_min, AABB_obj_max])
        
        return AABB_obj
    
    # TODO: implement a* algorithm                
    
    def world_to_grid(self, world_pos):
        """
        Convert world coordinates (x, y) to grid coordinates.
        """
        x, y = world_pos
        grid_x = int((x - self.x_min) / self.grid_resolution)
        grid_y = int((y - self.y_min) / self.grid_resolution)
        return grid_x, grid_y
    
    def is_occupied(self, x, y, goal_id=None, robot_id=None, robot_z_range=None):
        """
        check if is collision free universally or for specific object
        """
        if 0 <= x < self.grid_size_x and 0 <= y < self.grid_size_y:
            node = self.map[x][y]
            objects_in_cell = node.get_objects()
            
            # no id and range provided, means check universal occupation
            if goal_id is None or robot_id is None:
                return bool(objects_in_cell)
            
            # Compare the robot's z-range with objects' z-ranges in the cell
            robot_min_z, robot_max_z = robot_z_range
            for obj_id, (obj_min_z, obj_max_z) in objects_in_cell.items():
                if obj_id in [robot_id, goal_id]:
                    continue
                if not (robot_max_z < obj_min_z or robot_min_z > obj_max_z):
                    return True

        return False  # No object or no z-range collision, the cell is free
    
    def get_heuristic(self, node1, node2):
        """
        Calculate the heuristic (Euclidean distance) between two nodes.
        """
        
        # Penalty for proximity to obstacles
        penalty = 0
        proximity_threshold = 2  # Number of grid cells considered 'near' an obstacle
        
        for i in range(-proximity_threshold, proximity_threshold + 1):
            for j in range(-proximity_threshold, proximity_threshold + 1):
                nx, ny = node1.x + i, node1.y + j
                if 0 <= nx < self.grid_size_x and 0 <= ny < self.grid_size_y and self.is_occupied(nx, ny):
                    distance_to_obstacle = np.hypot(i, j)
                    penalty += max(0, proximity_threshold - distance_to_obstacle)  # Apply higher penalty for closer obstacles
        
        return np.hypot(node1.x - node2.x, node1.y - node2.y) + penalty
    
    def reconstruct_path(self, current, closed_set):
        path = []
        while current is not None:
            path.append((current.x, current.y))
            current = closed_set.get(current.parent_index)
        return path[::-1]  # Return reversed path
  
    def get_astar_map(self, robot_id, goal_id):
        # Get robot's center position (from AABB)
        robot_aabb = self.getAABB(robot_id)
        min_x, min_y, min_z = robot_aabb[0]
        max_x, max_y, max_z = robot_aabb[1]
        robot_center = ((min_x + max_x)/2, (min_y + max_y)/2)
        robot_z_range = (min_z, max_z)
        
        # Get goal's center position (from AABB)
        goal_aabb = self.getAABB(goal_id)
        goal_center = (
            (goal_aabb[0][0] + goal_aabb[1][0]) / 2,  # center_x
            (goal_aabb[0][1] + goal_aabb[1][1]) / 2   # center_y
        )
        
        # Convert world coordinates to grid coordinates
        start_x, start_y = self.world_to_grid(robot_center)
        goal_x, goal_y = self.world_to_grid(goal_center)
        
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
            (start_node.cost + self.get_heuristic(start_node, goal_node), (start_node.x, start_node.y))
        )
        
        while pq:
            _, current_coord = heapq.heappop(pq)
                        
            # Skip outdated nodes
            if current_coord not in open_set:
                continue  
            
            current = open_set[current_coord]
            node_objects = self.map[current.x][current.y].get_objects()
            
            # get node info to check if reach the goal
            if goal_id in node_objects:
                print("Path found!")
                return self.reconstruct_path(current, visited)
            
            # update information
            del open_set[current_coord]
            visited[current_coord] = current
            
            # Explore neighbors (8 grids nearby)
            for action in self.actions:
                new_x = current.x + action[0]
                new_y = current.y + action[1]
                new_cost = current.cost + action[2] # travel length
                
                # Check if within bounds
                if new_x < 0 or new_x >= self.grid_size_x or new_y < 0 or new_y >= self.grid_size_y:
                    continue
                
                # skip if is already visited 
                if (new_x, new_y) in visited:
                    continue
                
                new_node = AStarNode(new_x, new_y, new_cost, (current.x, current.y))

                # check if available for robot to move
                if self.is_occupied(new_x, new_y, goal_id, robot_id, robot_z_range):
                    continue  
                
                # If node is new or has a better path, add it to open set
                if (new_x, new_y) not in open_set or \
                    open_set[(new_x, new_y)].cost > new_node.cost:
                    
                    open_set[(new_x, new_y)] = new_node
                    heapq.heappush(
                        pq, 
                        (new_node.cost + self.get_heuristic(new_node, goal_node), (new_x, new_y))
                    )

        # Return None if no path is found
        print("No available path found.")
        return None
    
    def visualize_astar(self, path, robot_id, goal_id):
        fig, ax = plt.subplots(figsize=(8, 8))

        # Create a color map for different objects, similar to show_map
        unique_objects = set()
        for i in range(self.grid_size_x):
            for j in range(self.grid_size_y):
                unique_objects.update(self.map[i][j].get_objects().keys())

        # Use a color map to assign a unique color to each object
        colormap = plt.get_cmap('tab10')
        colors = {obj_id: colormap(i % 10) for i, obj_id in enumerate(unique_objects)}

        # Draw grid cells with plt.Rectangle and mark objects using scatter
        for i in range(self.grid_size_x):
            for j in range(self.grid_size_y):
                node = self.map[i][j]
                objects = node.get_objects()
                x = i  # Rectangle grid coordinate
                y = j
                # Add a rectangle for the cell
                ax.add_patch(plt.Rectangle((x, y), 1, 1, color='lightgray', edgecolor='gray', alpha=0.5))

                # If the node has objects, scatter points in the center of the grid cell
                for obj_id in objects:
                    ax.scatter(x + 0.5, y + 0.5, color=colors[obj_id], s=50)

        # Draw the A* path (if found)
        if path:
            path_x, path_y = zip(*path)
            ax.plot([x + 0.5 for x in path_x], [y + 0.5 for y in path_y], color='blue', linewidth=2, label='A* Path')

        # Mark the robot position (green circle)
        robot_aabb = self.getAABB(robot_id)
        robot_center = (
            (robot_aabb[0][0] + robot_aabb[1][0]) / 2,
            (robot_aabb[0][1] + robot_aabb[1][1]) / 2
        )
        robot_x, robot_y = self.world_to_grid(robot_center)
        ax.scatter(robot_x + 0.5, robot_y + 0.5, color='green', s=100, label='Robot', marker='o')

        # Mark the goal position (red star)
        goal_aabb = self.getAABB(goal_id)
        goal_center = (
            (goal_aabb[0][0] + goal_aabb[1][0]) / 2,
            (goal_aabb[0][1] + goal_aabb[1][1]) / 2
        )
        goal_x, goal_y = self.world_to_grid(goal_center)
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
        ax.set_title('A* Path Visualization on 2D Grid')
        ax.grid(True)

        ax.legend(loc='upper right', bbox_to_anchor=(1.2, 1), title="Legend")
        
        plt.gca().set_aspect('equal', adjustable='box')
        plt.tight_layout()
        plt.show()

      
                