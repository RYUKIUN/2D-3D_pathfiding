import random
import heapq

class Node:
    def __init__(self, x, y, g, h, parent=None):
        self.x = x
        self.y = y
        self.g = g  # Cost from start to this node
        self.h = h  # Heuristic cost to the goal
        self.f = g + h  # Total cost (f = g + h)
        self.parent = parent  # Parent node for path reconstruction

    def __lt__(self, other):
        return self.f < other.f  # Compare nodes based on their f value

def heuristic(a, b):
    """Heuristic for A* (Manhattan distance)"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def is_within_boundaries(x, y, space_corners):
    """Check if a point is within the defined boundaries."""
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = space_corners
    min_x = min(x1, x2, x3, x4)
    max_x = max(x1, x2, x3, x4)
    min_y = min(y1, y2, y3, y4)
    max_y = max(y1, y2, y3, y4)
    return min_x <= x < max_x and min_y <= y < max_y

def is_obstacle(x, y, obstacles):
    """Check if a point is an obstacle."""
    return (x, y) in obstacles

def is_space_free(x, y, host_size, obstacles):
    """Check if the space for the object (defined by host_size) is free of obstacles."""
    width, height = host_size
    for dx in range(width):
        for dy in range(height):
            if (x + dx, y + dy) in obstacles:
                return False  # If any part of the host overlaps with an obstacle, space is not free
    return True  # Space is free for the whole object

def get_neighbors_with_host_size(node, space_corners, host_size, obstacles, movement_rules):
    """Return neighboring nodes considering the host size and obstacles."""
    neighbors = []
    width, height = host_size
    
    for dx, dy in movement_rules:
        new_x = node.x + dx * square_size
        new_y = node.y + dy * square_size
        
        # Check if the new position is within boundaries and if the whole space for the object is free
        if is_within_boundaries(new_x, new_y, space_corners) and is_space_free(new_x, new_y, host_size, obstacles):
            neighbors.append((new_x, new_y))
    
    return neighbors

def reconstruct_path(node):
    """Reconstruct the path by backtracking from the goal node."""
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]  # Reverse the path

def generate_obstacles(obstacle_spaces, obstacle_density=0.3):
    """Generate obstacles within multiple defined spaces.
    
    obstacle_spaces: A list of bounding boxes or regions [(x_min, y_min, x_max, y_max), ...].
    obstacle_density: A float (0 to 1) indicating how densely to populate each region with obstacles.
    """
    obstacles = []
    
    for obstacle_space in obstacle_spaces:
        x_min, y_min, x_max, y_max = obstacle_space
        total_cells = (x_max - x_min + 1) * (y_max - y_min + 1)
        num_obstacles = int(total_cells * obstacle_density)
        
        # Generate obstacles within this specific obstacle region
        generated_obstacles = set()  # Use a set to avoid duplicates
        while len(generated_obstacles) < num_obstacles:
            obstacle = (random.randint(x_min, x_max), random.randint(y_min, y_max))
            generated_obstacles.add(obstacle)
        
        obstacles.extend(generated_obstacles)  # Add the generated obstacles to the main list
    
    return obstacles

def a_star_pathfinding(space_corners, square_size, start, end, obstacles, movement_rules, host_size):
    """A* Pathfinding algorithm with obstacles and custom movement rules."""
    open_list = []
    closed_list = set()

    start_node = Node(start[0], start[1], 0, heuristic(start, end))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        
        # If we reached the goal
        if (current_node.x, current_node.y) == end:
            return reconstruct_path(current_node)

        closed_list.add((current_node.x, current_node.y))

        # Check neighboring nodes based on movement rules
        for neighbor in get_neighbors_with_host_size(current_node, space_corners, host_size, obstacles, movement_rules):
            if (neighbor[0], neighbor[1]) in closed_list:
                continue

            g_cost = current_node.g + 1  # Each move has a cost of 1
            h_cost = heuristic(neighbor, end)
            neighbor_node = Node(neighbor[0], neighbor[1], g_cost, h_cost, current_node)

            # Add the neighbor to the open list if not already there or if it's a better path
            if not any((n.x, n.y) == (neighbor_node.x, neighbor_node.y) and n.f <= neighbor_node.f for n in open_list):
                heapq.heappush(open_list, neighbor_node)

    return None  # No path found

# Example usage:
space_corners = [(0, 0), (20, 0), (0, 20), (20, 20)]  # Define the 4 corners of the space
square_size = 1  # Size of the grid squares
start = (1, 1)  # Start coordinate
end = (19, 15)    # End coordinate
host_size = (4, 4)  # Height and width of the object
obstacle_spaces = [(5, 5, 10, 10), (14, 14, 16, 16)]  # Multiple obstacle regions
obstacles = generate_obstacles(obstacle_spaces, obstacle_density=1)  # Generate obstacles
movement_rules = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]  # 8-way movement

print("Generated Obstacles:", obstacles)  # Optional: Print the obstacles for reference

path = a_star_pathfinding(space_corners, square_size, start, end, obstacles, movement_rules, host_size)
if path:
    print("Path found:")
    for step in path:
        print(step)
else:
    print("No path found.")
