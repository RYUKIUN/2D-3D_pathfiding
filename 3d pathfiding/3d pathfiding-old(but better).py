import random
import heapq

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
import numpy as np

class Node:
    def __init__(self, x, y, z, g, h, parent=None):
        self.x = x
        self.y = y
        self.z = z
        self.g = g  # Cost from start to this node
        self.h = h  # Heuristic cost to the goal
        self.f = g + h  # Total cost (f = g + h)
        self.parent = parent  # Parent node for path reconstruction

    def __lt__(self, other):
        return self.f < other.f  # Compare nodes based on their f value

def heuristic(a, b):
    """Heuristic for A* (Manhattan distance in 3D)"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])

def is_within_boundaries(x, y, z, space_corners):
    """Check if a point is within the defined 3D boundaries."""
    (x1, y1, z1), (x2, y2, z2), (x3, y3, z3), (x4, y4, z4), \
    (x5, y5, z5), (x6, y6, z6), (x7, y7, z7), (x8, y8, z8) = space_corners
    min_x = min(x1, x2, x3, x4, x5, x6, x7, x8)
    max_x = max(x1, x2, x3, x4, x5, x6, x7, x8)
    min_y = min(y1, y2, y3, y4, y5, y6, y7, y8)
    max_y = max(y1, y2, y3, y4, y5, y6, y7, y8)
    min_z = min(z1, z2, z3, z4, z5, z6, z7, z8)
    max_z = max(z1, z2, z3, z4, z5, z6, z7, z8)
    return min_x <= x < max_x and min_y <= y < max_y and min_z <= z < max_z

def is_obstacle(x, y, z, obstacles):
    """Check if a 3D point is an obstacle."""
    return (x, y, z) in obstacles

def is_space_free(x, y, z, host_size, obstacles):
    """Check if the space for the object (defined by host_size) is free of obstacles."""
    width, height, depth = host_size
    for dx in range(width):
        for dy in range(height):
            for dz in range(depth):
                if (x + dx, y + dy, z + dz) in obstacles:
                    return False  # If any part of the host overlaps with an obstacle, space is not free
    return True  # Space is free for the whole object

def get_neighbors_with_host_size(node, space_corners, host_size, obstacles, movement_rules):
    """Return neighboring nodes in 3D considering the host size and obstacles."""
    neighbors = []
    width, height, depth = host_size
    
    for dx, dy, dz in movement_rules:
        new_x = node.x + dx
        new_y = node.y + dy
        new_z = node.z + dz
        
        # Check if the new position is within boundaries and if the whole space for the object is free
        if is_within_boundaries(new_x, new_y, new_z, space_corners) and is_space_free(new_x, new_y, new_z, host_size, obstacles):
            neighbors.append((new_x, new_y, new_z))
    
    return neighbors

def reconstruct_path(node):
    """Reconstruct the path by backtracking from the goal node."""
    path = []
    while node:
        path.append((node.x, node.y, node.z))
        node = node.parent
    return path[::-1]  # Reverse the path

def generate_obstacles(obstacle_spaces, obstacle_density=0.3):
    obstacles = []

    for obstacle_space in obstacle_spaces:
        x_min, y_min, z_min, x_max, y_max, z_max = obstacle_space
        total_cells = (x_max - x_min + 1) * (y_max - y_min + 1) * (z_max - z_min + 1)
        num_obstacles = int(total_cells * obstacle_density)
        
        # Calculate a step size for evenly distributed obstacles
        x_range = x_max - x_min + 1
        y_range = y_max - y_min + 1
        z_range = z_max - z_min + 1
        
        step = int((total_cells / num_obstacles) ** (1 / 3))  # Approximate spacing in each dimension

        # Iterate over the 3D grid with the calculated step
        for x in range(x_min, x_max + 1, max(1, step)):
            for y in range(y_min, y_max + 1, max(1, step)):
                for z in range(z_min, z_max + 1, max(1, step)):
                    if len(obstacles) < num_obstacles:
                        obstacles.append((x, y, z))
                    else:
                        break
    
    return obstacles


def a_star_pathfinding(space_corners, square_size, start, end, obstacles, movement_rules, host_size):
    """A* Pathfinding algorithm with obstacles and custom movement rules."""
    open_list = []
    closed_list = set()

    start_node = Node(start[0], start[1], start[2], 0, heuristic(start, end))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        
        # If we reached the goal
        if (current_node.x, current_node.y, current_node.z) == end:
            return reconstruct_path(current_node)

        closed_list.add((current_node.x, current_node.y, current_node.z))

        # Check neighboring nodes based on movement rules
        for neighbor in get_neighbors_with_host_size(current_node, space_corners, host_size, obstacles, movement_rules):
            if (neighbor[0], neighbor[1], neighbor[2]) in closed_list:
                continue

            g_cost = current_node.g + 1  # Each move has a cost of 1
            h_cost = heuristic(neighbor, end)
            neighbor_node = Node(neighbor[0], neighbor[1], neighbor[2], g_cost, h_cost, current_node)

            # Add the neighbor to the open list if not already there or if it's a better path
            if not any((n.x, n.y, n.z) == (neighbor_node.x, neighbor_node.y, neighbor_node.z) and n.f <= neighbor_node.f for n in open_list):
                heapq.heappush(open_list, neighbor_node)

    return None  # No path found

def find_path_with_waypoints(space_corners, square_size, start, waypoints, end, obstacles, movement_rules, host_size):
    """Find a path with multiple waypoints in 3D space."""
    path = []
    current_start = start

    for waypoint in waypoints:
        segment_path = a_star_pathfinding(space_corners, square_size, current_start, waypoint, obstacles, movement_rules, host_size)
        if segment_path:
            path.extend(segment_path[:-1])  # Add all but the last point (to avoid duplicates)
            current_start = waypoint
        else:
            return None  # No valid path through waypoints

    final_path = a_star_pathfinding(space_corners, square_size, current_start, end, obstacles, movement_rules, host_size)
    if final_path:
        path.extend(final_path)  # Add the final path
        return path

    return None  # No path found


# Helper function to draw rectangular prisms as obstacles
def draw_obstacles(ax, obstacles):
    for obstacle in obstacles:
        xmin, ymin, zmin, xmax, ymax, zmax = obstacle
        vertices = [
            [xmin, ymin, zmin], [xmin, ymax, zmin], [xmax, ymax, zmin], [xmax, ymin, zmin],  # Bottom face
            [xmin, ymin, zmax], [xmin, ymax, zmax], [xmax, ymax, zmax], [xmax, ymin, zmax]   # Top face
        ]
        faces = [
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Left face
            [vertices[1], vertices[2], vertices[6], vertices[5]],  # Front face
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Right face
            [vertices[3], vertices[0], vertices[4], vertices[7]],  # Back face
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top face
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom face
        ]
        prism = Poly3DCollection(faces, color='gray', alpha=0.6)
        ax.add_collection3d(prism)

# Helper function to draw the host
def draw_host(ax, pos, host_size):
    x, y, z = pos
    width, height, depth = host_size
    return ax.bar3d(x, y, z, width, height, depth, color='blue', alpha=0.6)

# Function to draw waypoints
def draw_waypoints(ax, waypoints):
    for waypoint in waypoints:
        ax.scatter(waypoint[0], waypoint[1], waypoint[2], color='orange', s=100, label="Waypoint", alpha=0.7)

# Function to draw the wireframe boundary of the space using space_corners
def draw_space_boundary(ax, space_corners):
    edges = [
        [space_corners[0], space_corners[1]], [space_corners[1], space_corners[3]], [space_corners[3], space_corners[2]], [space_corners[2], space_corners[0]],  # Bottom face
        [space_corners[4], space_corners[5]], [space_corners[5], space_corners[7]], [space_corners[7], space_corners[6]], [space_corners[6], space_corners[4]],  # Top face
        [space_corners[0], space_corners[4]], [space_corners[1], space_corners[5]], [space_corners[2], space_corners[6]], [space_corners[3], space_corners[7]]  # Vertical edges
    ]
    for edge in edges:
        ax.plot3D(*zip(*edge), color='black')

# Visualization function to simulate the host movement along the predefined path
def visualize_movement_3d(space_corners, start, end, host_size, obstacles, path, waypoints, delay=0.5):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_axis_off()

    ax.set_xlim([min(c[0] for c in space_corners), max(c[0] for c in space_corners)])
    ax.set_ylim([min(c[1] for c in space_corners), max(c[1] for c in space_corners)])
    ax.set_zlim([min(c[2] for c in space_corners), max(c[2] for c in space_corners)])

    draw_space_boundary(ax, space_corners)
    ax.scatter(start[0], start[1], start[2], color='green', s=100, label="Start")
    ax.scatter(end[0], end[1], end[2], color='red', s=100, label="End")
    draw_obstacles(ax, obstacles)
    draw_waypoints(ax, waypoints)

    # Plot the path as a line
    path_points = np.array(path)
    ax.plot(path_points[:, 0], path_points[:, 1], path_points[:, 2], color='blue', linestyle='-', linewidth=1, label="Path")

    host_ref = draw_host(ax, start, host_size)

    def update(frame):
        nonlocal host_ref
        host_ref.remove()
        current_pos = path[frame]
        host_ref = draw_host(ax, current_pos, host_size)

    anim = FuncAnimation(fig, update, frames=len(path), interval=delay*1000, blit=False, repeat=False)

    plt.legend()
    plt.show()

# Example usage:
space_corners = [(0, 0, 0), (0, 0, 100), (0, 100, 0),
 (0, 100, 100), (100, 0, 0), (100, 0, 100), (100, 100, 0),
  (100, 100, 100)]
square_size = 1  # Size of the grid cubes
start = (5, 5, 5)  # Start coordinate
end = (90, 90, 90)    # End coordinate
host_size = (3, 3, 3)  # Height, width, and depth of the object
obstacle_spaces = [(30,1,1,35,80,100),(60,20,1,65,100,100)]  # Multiple obstacle regions
obstacles = generate_obstacles(obstacle_spaces, obstacle_density=1)  # Generate obstacles
waypoints = [(33,90,40),(63,10,60)]  # Define waypoints
movement_rules = [(1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
    
    (1, 1, 0),
    (1, -1, 0),
    (-1, 1, 0),
    (-1, -1, 0),
    
    (1, 0, 1),
    (1, 0, -1),
    (-1, 0, 1),
    (-1, 0, -1),
    
    (0, 1, 1),
    (0, 1, -1),
    (0, -1, 1),
    (0, -1, -1),
    
    (1, 1, 1),
    (1, 1, -1),
    (1, -1, 1),
    (1, -1, -1),
    (-1, 1, 1),
    (-1, 1, -1),
    (-1, -1, 1),
    (-1, -1, -1)]

print("Generated Obstacles:", obstacles)  # Optional: Print the obstacles for reference

# Main code for pathfinding and rendering
path = find_path_with_waypoints(space_corners, square_size, start, waypoints, end, obstacles, movement_rules, host_size)

if path:
    print("Path found through waypoints:")
    for step in path:
        print(step, end=", ")
    
    # Ask if the user wants to render the path
    render_path = input("\nDo you want to visualize the path? (yes/no): ").strip().lower()
    if render_path in ['yes', 'y']:
        visualize_movement_3d(space_corners, start, end, host_size, obstacle_spaces, path, waypoints, delay=0.5)
    else:
        print("Visualization skipped.")
else:
    print("No path found.")
