import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import random

def generate_obstacles(obstacle_spaces, obstacle_density=0.3):
    """Generate obstacles within multiple defined spaces."""
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

# Helper function to draw obstacles
def draw_obstacles(ax, obstacle_spaces, obstacles):
    for obstacle in obstacles:
        rect = patches.Rectangle((obstacle[0], obstacle[1]), 1, 1, linewidth=1, edgecolor='black', facecolor='gray')
        ax.add_patch(rect)

# Helper function to draw the host
def draw_host(ax, pos, host_size):
    width, height = host_size
    rect = patches.Rectangle((pos[0], pos[1]), width, height, linewidth=2, edgecolor='blue', facecolor='none')
    return rect

# Visualization function to simulate the host movement along the predefined path
def visualize_movement(space_corners, start, end, host_size, obstacle_spaces, obstacles, path, delay=0.5):
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # Set limits based on space_corners
    min_x = min([corner[0] for corner in space_corners])
    max_x = max([corner[0] for corner in space_corners])
    min_y = min([corner[1] for corner in space_corners])
    max_y = max([corner[1] for corner in space_corners])
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)

    # Draw start and end points
    ax.plot(start[0], start[1], "go", label="Start")  # Green start point
    ax.plot(end[0], end[1], "ro", label="End")  # Red end point

    # Draw obstacles
    draw_obstacles(ax, obstacle_spaces, obstacles)

    # Initialize the host as a rectangle
    host_rect = draw_host(ax, start, host_size)
    ax.add_patch(host_rect)

    # Animation function to update host position along the path
    def update(frame):
        # Move the host rectangle to the next position in the path
        current_pos = path[frame]
        host_rect.set_xy((current_pos[0], current_pos[1]))
        return host_rect,

    # Create the animation
    anim = FuncAnimation(fig, update, frames=len(path), interval=delay*500, blit=True, repeat=False)

    # Show the legend and plot
    plt.legend()
    plt.show()

# Example input data for visualization (no pathfinding calculation here)
space_corners = [(0, 0), (100, 0), (0, 100), (100, 100)]  # Define the 4 corners of the space
start = (5, 5)  # Start coordinate
end = (10, 90)  # End coordinate
host_size = (6, 6)  # Height and width of the object
obstacle_spaces = [(0,0,100,0),(0,0,0,100),(0,100,100,100),(100,0,100,100),(10,5,20,10),(10,30,20,50),(10,70,20,80)]  # Multiple obstacle regions
obstacles = generate_obstacles(obstacle_spaces, obstacle_density=1)

# Predefined path for the host to walk (example)
path = [(4, 6),(4, 7),(4, 8),(4, 9),(4, 10),(5, 11),(6, 12),(7, 13),(8, 14),(9, 15),(10, 16),(11, 17),(12, 18),(13, 19),(14, 20),(15, 20),(16, 21),(17, 22),(18, 23),(19, 24),(20, 24),(21, 25),(21, 26),(21, 27),(21, 28),(21, 29),(21, 
30),(21, 31),(21, 32),(21, 33),(21, 34),(21, 35),(21, 36),(21, 37),(21, 38),(21, 39),(21, 40),(21, 41),(21, 42),(21, 43),(21, 44),(21, 45),(21, 46),(21, 47),(21, 48),(21, 49),(21, 50),(20, 51),(19, 52),(18, 53),(17, 54),(16, 
55),(15, 56),(15, 57),(15, 58),(15, 59),(15, 60),(14, 61),(13, 62),(12, 63),(11, 64),(10, 64),(9, 64),(8, 64),(7, 64),(6, 64),(5, 64),(4, 65),(4, 66),(4, 67),(4, 68),(4, 69),(4, 70),(4, 71),(4, 72),(4, 73),(4, 74),(4, 75),(4, 76),(4, 77),(4, 78),(4, 79),(4, 80),(5, 81),(6, 82),(7, 83),(8, 84),(9, 85),(10, 86),(10, 87),(10, 88),(10, 89),(10, 90)]

# Simulate movement along the predefined path
visualize_movement(space_corners, start, end, host_size, obstacle_spaces, obstacles, path, delay=0.5)
