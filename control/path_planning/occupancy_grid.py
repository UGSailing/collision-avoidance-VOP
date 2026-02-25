import numpy as np
import matplotlib.pyplot as plt

# 1. Grid Parameters
width_m = 10.0  # meters
height_m = 10.0 # meters
resolution = 0.1 # meters per cell

rows = int(height_m / resolution)
cols = int(width_m / resolution)

# 2. Initialize grid (0.5 for unknown, or 0 for empty)
occupancy_grid = np.zeros((rows, cols))

# 3. Add fake obstacles (static shapes)
# Let's put a wall at x = 5m
wall_idx = int(5.0 / resolution)
occupancy_grid[wall_idx:wall_idx+2, 20:80] = 1.0

# Let's put a square pillar at (2m, 2m)
p_x, p_y = int(2.0/resolution), int(2.0/resolution)
occupancy_grid[p_x:p_x+5, p_y:p_y+5] = 1.0

# 4. Visualize
plt.imshow(occupancy_grid, cmap='Greys', origin='lower')
plt.title("Test Occupancy Grid (10x10m)")
plt.xlabel("X (cells)")
plt.ylabel("Y (cells)")
plt.show()