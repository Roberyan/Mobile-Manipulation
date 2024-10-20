import numpy as np
import matplotlib.pyplot as plt

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


class NavMap:
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
            
            obj_aabb = self.p.getAABB(obj_id)
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
           
    def label_boundary(self):
        # label boundary
        for y in range(self.grid_size_y):
            self.map[0][y].add_object(0, 0)
            self.map[self.grid_size_x-1][y].add_object(0, 0)
        
        for x in range(self.grid_size_x):
            self.map[x][0].add_object(0, 0)
            self.map[x][self.grid_size_y-1].add_object(0, 0)
    
    def label_objects(self, no_label_id):
        num_bodies = self.p.getNumBodies()
        object_ids = [self.p.getBodyUniqueId(i) for i in range(num_bodies)]
        for obj_id in object_ids:
            if obj_id in no_label_id:
                continue
            self.map.add_object(obj_id)
    
    def add_object(self, object_id):
        obj_aabb = self.p.getAABB(object_id)
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
        colormap = plt.get_cmap('tab10')  # You can choose other colormaps like 'hsv', 'tab20', etc.
        colors = {obj_id: colormap(i % 10) for i, obj_id in enumerate(unique_objects)}
        
        # Draw the grid lines
        for i in range(self.grid_size_x + 1):
            x = self.x_min + i * self.grid_resolution
            ax.plot([x, x], [self.y_min, self.y_min + self.grid_size_y * self.grid_resolution], color='gray', linewidth=0.5)
        for j in range(self.grid_size_y + 1):
            y = self.y_min + j * self.grid_resolution
            ax.plot([self.x_min, self.x_min + self.grid_size_x * self.grid_resolution], [y, y], color='gray', linewidth=0.5)

        # Mark the objects in the grid
        for i in range(self.grid_size_x):
            for j in range(self.grid_size_y):
                node = self.map[i][j]
                # If the node has objects, plot them as points in the grid cell
                objects = node.get_objects()
                for obj_id in objects:
                    x = self.x_min + i * self.grid_resolution + self.grid_resolution / 2.0  # Center of the cell in x
                    y = self.y_min + j * self.grid_resolution + self.grid_resolution / 2.0  # Center of the cell in y
                    ax.scatter(x, y, color=colors[obj_id], s=50, label=f'Object {obj_id}' if i + j == 0 else "")

        # Set axis limits and labels
        ax.set_xlim([self.x_min, self.x_min + self.grid_size_x * self.grid_resolution])
        ax.set_ylim([self.y_min, self.y_min + self.grid_size_y * self.grid_resolution])
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.set_title("2D Grid Map with Objects")
        ax.grid(True)

        # Create a custom legend to show the object ID with corresponding colors
        handles = [plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=colors[obj_id], markersize=10, label=f'Object {obj_id}') for obj_id in unique_objects]
        ax.legend(handles=handles, loc='upper right')

        # Show the plot
        plt.show()
                    
    # def get_astar_map(self):
        