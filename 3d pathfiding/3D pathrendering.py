import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FuncAnimation
import numpy as np

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

# Example input data for visualization
space_corners = [
    (0, 0, 0), (0, 0, 100), (0, 100, 0), (0, 100, 100),
    (100, 0, 0), (100, 0, 100), (100, 100, 0), (100, 100, 100)
]
start = (5, 5, 5)
end = (90, 90, 90)
host_size = (3, 3, 3)
obstacles = [(30,1,1,35,80,100),(60,20,1,65,100,100)]
path = [(5, 5, 5),(6, 6, 6),(7, 7, 7),(8, 8, 8),(9, 9, 9),(10, 10, 10),(11, 11, 11),(12, 12, 12),(13, 13, 13),(14, 14, 14),(15, 15, 15),(16, 
16, 16),(17, 17, 17),(18, 18, 18),(19, 19, 19),(20, 20, 20),(21, 21, 21),(22, 22, 22),(23, 23, 23),(24, 24, 24),(25, 25, 25),(26, 26, 26),(27, 27, 27),(27, 28, 28),(27, 29, 29),(27, 30, 30),(27, 31, 31),(27, 32, 32),(27, 33, 33),(27, 34, 34),(27, 35, 35),(27, 36, 36),(27, 37, 37),(27, 38, 38),(27, 39, 39),(27, 40, 40),(27, 41, 40),(27, 42, 40),(27, 43, 40),(27, 44, 40),(27, 45, 40),(27, 46, 40),(27, 47, 40),(27, 48, 40),(27, 49, 40),(27, 50, 40),(27, 51, 40),(27, 52, 40),(27, 53, 40),(27, 54, 40),(27, 55, 40),(27, 56, 40),(27, 57, 40),(27, 58, 40),(27, 59, 40),(27, 60, 40),(27, 61, 40),(27, 62, 40),(27, 63, 40),(27, 64, 40),(27, 65, 40),(27, 66, 40),(27, 67, 40),(27, 68, 40),(27, 69, 40),(27, 70, 40),(27, 71, 40),(27, 72, 40),(27, 73, 40),(27, 74, 40),(27, 75, 40),(27, 76, 40),(27, 77, 40),(27, 78, 40),(27, 79, 40),(27, 80, 40),(28, 81, 40),(29, 82, 40),(30, 83, 40),(31, 84, 40),(32, 85, 40),(33, 86, 40),(33, 87, 40),(33, 88, 40),(33, 89, 40),(33, 90, 40),(34, 89, 41),(35, 88, 42),(36, 87, 43),(37, 86, 44),(38, 85, 45),(39, 84, 46),(40, 83, 47),(41, 82, 48),(42, 81, 49),(43, 80, 50),(44, 79, 51),(45, 78, 52),(46, 77, 53),(47, 76, 54),(48, 75, 55),(49, 74, 56),(50, 73, 57),(51, 72, 58),(52, 71, 59),(53, 70, 60),(54, 69, 60),(55, 68, 60),(56, 67, 60),(57, 66, 60),(57, 65, 60),(57, 64, 60),(57, 63, 60),(57, 62, 
60),(57, 61, 60),(57, 60, 60),(57, 59, 60),(57, 58, 60),(57, 57, 60),(57, 56, 60),(57, 55, 60),(57, 54, 60),(57, 53, 60),(57, 52, 60),(57, 51, 60),(57, 50, 60),(57, 49, 60),(57, 48, 60),(57, 47, 60),(57, 46, 60),(57, 45, 60),(57, 44, 60),(57, 43, 60),(57, 42, 60),(57, 41, 60),(57, 40, 60),(57, 39, 60),(57, 38, 60),(57, 37, 60),(57, 36, 60),(57, 35, 60),(57, 34, 60),(57, 33, 60),(57, 32, 60),(57, 
31, 60),(57, 30, 60),(57, 29, 60),(57, 28, 60),(57, 27, 60),(57, 26, 60),(57, 25, 60),(57, 24, 60),(57, 23, 60),(57, 22, 60),(57, 21, 60),(57, 20, 60),(57, 19, 60),(57, 18, 60),(58, 17, 60),(59, 16, 60),(60, 15, 60),(61, 14, 60),(62, 13, 60),(63, 12, 60),(63, 11, 60),(63, 10, 60),(64, 11, 61),(65, 12, 62),(66, 13, 63),(67, 14, 64),(68, 15, 65),(69, 16, 66),(70, 17, 67),(71, 18, 68),(72, 19, 69),(73, 20, 70),(74, 21, 71),(75, 22, 72),(76, 23, 73),(77, 24, 74),(78, 25, 75),(79, 26, 76),(80, 27, 77),(81, 28, 78),(82, 29, 79),(83, 30, 80),(84, 31, 81),(85, 32, 82),(86, 33, 83),(87, 34, 84),(88, 35, 85),(89, 36, 86),(90, 37, 87),(90, 38, 88),(90, 39, 89),(90, 40, 90),(90, 41, 90),(90, 42, 90),(90, 43, 90),(90, 44, 90),(90, 45, 90),(90, 46, 90),(90, 47, 90),(90, 48, 90),(90, 49, 90),(90, 50, 90),(90, 51, 90),(90, 52, 90),(90, 53, 90),(90, 54, 90),(90, 55, 90),(90, 56, 90),(90, 57, 90),(90, 58, 90),(90, 59, 90),(90, 60, 90),(90, 61, 90),(90, 62, 90),(90, 63, 90),(90, 64, 90),(90, 65, 90),(90, 66, 90),(90, 67, 90),(90, 68, 90),(90, 69, 90),(90, 70, 90),(90, 71, 90),(90, 72, 90),(90, 73, 90),(90, 74, 90),(90, 75, 90),(90, 76, 90),(90, 77, 90),(90, 78, 90),(90, 79, 90),(90, 80, 90),(90, 81, 90),(90, 82, 90),(90, 83, 90),(90, 84, 90),(90, 85, 90),(90, 86, 90),(90, 87, 90),(90, 88, 90),(90, 89, 90),(90, 90, 90)]
waypoints = [(33,90,40),(63,10,60)]

visualize_movement_3d(space_corners, start, end, host_size, obstacles, path, waypoints, delay=0.15)
