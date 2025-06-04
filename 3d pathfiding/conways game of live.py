import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Define the size of the grid
grid_size = 200

# Initialize the grid with random states
def initialize_grid(size):
    return np.random.choice([0, 1], size=(size, size), p=[0.8, 0.2])

# Update the grid according to the rules of the Game of Life
def update_grid(grid):
    new_grid = grid.copy()
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            # Count live neighbors
            total = int((grid[i, (j-1)%grid_size] + grid[i, (j+1)%grid_size] +
                          grid[(i-1)%grid_size, j] + grid[(i+1)%grid_size, j] +
                          grid[(i-1)%grid_size, (j-1)%grid_size] + grid[(i-1)%grid_size, (j+1)%grid_size] +
                          grid[(i+1)%grid_size, (j-1)%grid_size] + grid[(i+1)%grid_size, (j+1)%grid_size]))
            
            # Apply Conway's rules
            if grid[i, j] == 1:  # Cell is currently alive
                if total < 2 or total > 3:
                    new_grid[i, j] = 0  # Cell dies
            else:  # Cell is currently dead
                if total == 3:
                    new_grid[i, j] = 1  # Cell becomes alive
    return new_grid

# Animation update function
def animate(i):
    global grid
    grid = update_grid(grid)
    mat.set_data(grid)
    return [mat]

# Set up the figure and axis
fig, ax = plt.subplots()
grid = initialize_grid(grid_size)
mat = ax.matshow(grid, cmap='binary')
plt.axis('off')

# Create the animation
ani = animation.FuncAnimation(fig, animate, frames=200, interval=0, blit=True)
plt.show()
