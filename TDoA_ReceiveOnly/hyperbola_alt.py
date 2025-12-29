import numpy as np
from scipy.optimize import least_squares
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Known positions of base stations
base_stations = np.array([
    [0, 0, 0],       # Base Station 1
    [0, 4.5, 0],     # Base Station 2
    [4.5, 0, 0],     # Base Station 3
    [4.5, 4.5, 0]    # Base Station 4
])

# TDOA measurements in meters
tdoa_measurements = {
    (0, 1): 1.1883571347171973,    # TDOA between P1 and P2
    (0, 2): -0.43786848335078016,  # TDOA between P1 and P3
    (0, 3): 0.5339697258700264,    # TDOA between P1 and P4
    (1, 2): -1.5719249805119964,   # TDOA between P2 and P3
    (1, 3): -0.6784681165078089,   # TDOA between P2 and P4
    (2, 3): 0.9100941442702047     # TDOA between P3 and P4
}

def objective_function(position, base_stations, tdoa_measurements):
    differences = []
    
    for (i, j), tdoa in tdoa_measurements.items():
        dist_i = np.linalg.norm(position - base_stations[i])
        dist_j = np.linalg.norm(position - base_stations[j])
        differences.append((dist_i - dist_j) - tdoa)
    
    return differences

# Initial guess for the tag's position
initial_guess = np.array([2, 2, 0])

# Perform least squares optimization
result = least_squares(objective_function, initial_guess, args=(base_stations, tdoa_measurements))

# Extract the optimized position
optimized_position = result.x
print("Optimized Position:", optimized_position)

# Create a grid of points for visualization
x = np.linspace(-10, 10, 200)
y = np.linspace(-10, 10, 200)
z = np.linspace(-10, 10, 200)
X, Y, Z = np.meshgrid(x, y, z)

# Calculate distances to the base stations
distances = [np.sqrt((X - bx)**2 + (Y - by)**2 + (Z - bz)**2) for bx, by, bz in base_stations]

# Create masks for the hyperboloids
masks = []
tolerance = 1e-3
for (i, j), tdoa in tdoa_measurements.items():
    delta_dist = distances[i] - distances[j]
    mask = np.abs(delta_dist - tdoa) < tolerance
    masks.append(mask)

# Plot the hyperboloids and base stations
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
colors = ['cyan', 'magenta', 'yellow', 'blue', 'green', 'red']
labels = ['Hyperboloid 1-2', 'Hyperboloid 1-3', 'Hyperboloid 1-4', 'Hyperboloid 2-3', 'Hyperboloid 2-4', 'Hyperboloid 3-4']

for mask, color, label in zip(masks, colors, labels):
    ax.scatter(X[mask], Y[mask], Z[mask], s=1, label=label, alpha=0.5, color=color)

# Plot the base stations
for i, (bx, by, bz) in enumerate(base_stations):
    ax.scatter(bx, by, bz, s=100, label=f'Base Station {i+1} ({bx}, {by}, {bz})')

# Plot the optimized position
ax.scatter(optimized_position[0], optimized_position[1], optimized_position[2], color='yellow', s=100, label='Optimized Position')

# Add labels and legend
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()
