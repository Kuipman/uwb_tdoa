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
tdoa_measurements = np.array([
    1.0274027996115447,   # TDOA between P1 and P2
    0.39525416704020455,  # TDOA between P1 and P3
    -0.5296863659009495   # TDOA between P1 and P4
])

# True position of the tag
true_position = np.array([2, 3, 0])

# Define the objective function
def objective_function(position, base_stations, tdoa_measurements):
    P1 = base_stations[0]
    differences = []
    
    for i, (P2, tdoa) in enumerate(zip(base_stations[1:], tdoa_measurements)):
        dist1 = np.linalg.norm(position - P1)
        dist2 = np.linalg.norm(position - P2)
        differences.append((dist1 - dist2) - tdoa)
    
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
dist1 = np.sqrt((X - base_stations[0][0])**2 + (Y - base_stations[0][1])**2 + (Z - base_stations[0][2])**2)
dist2 = np.sqrt((X - base_stations[1][0])**2 + (Y - base_stations[1][1])**2 + (Z - base_stations[1][2])**2)
dist3 = np.sqrt((X - base_stations[2][0])**2 + (Y - base_stations[2][1])**2 + (Z - base_stations[2][2])**2)
dist4 = np.sqrt((X - base_stations[3][0])**2 + (Y - base_stations[3][1])**2 + (Z - base_stations[3][2])**2)

# Calculate the difference in distances
delta_dist1 = dist1 - dist2
delta_dist2 = dist1 - dist3
delta_dist3 = dist1 - dist4

# Find points where the difference in distances matches the TDOA measurements
tolerance = 1e-3
mask1 = np.abs(delta_dist1 - tdoa_measurements[0]) < tolerance
mask2 = np.abs(delta_dist2 - tdoa_measurements[1]) < tolerance
mask3 = np.abs(delta_dist3 - tdoa_measurements[2]) < tolerance

# Plot the hyperboloids and base stations
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X[mask1], Y[mask1], Z[mask1], s=1, label='Hyperboloid 1')
ax.scatter(X[mask2], Y[mask2], Z[mask2], s=1, label='Hyperboloid 2', alpha=0.5)
ax.scatter(X[mask3], Y[mask3], Z[mask3], s=1, label='Hyperboloid 3', alpha=0.5)

# Plot the base stations
ax.scatter(base_stations[:, 0], base_stations[:, 1], base_stations[:, 2], color=['red', 'blue', 'green', 'orange'], s=100)
for i, label in enumerate(['Base Station 1 (0, 0, 0)', 'Base Station 2 (0, 4.5, 0)', 'Base Station 3 (4.5, 0, 0)', 'Base Station 4 (4.5, 4.5, 0)']):
    ax.text(base_stations[i, 0], base_stations[i, 1], base_stations[i, 2], label, color='black')

# Plot the true position
ax.scatter(true_position[0], true_position[1], true_position[2], color='purple', s=100, label='True Position (2, 3, 0)')

# Plot the optimized position
ax.scatter(optimized_position[0], optimized_position[1], optimized_position[2], color='yellow', s=100, label='Optimized Position')

# Add labels and legend
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()


# import numpy as np
# from scipy.optimize import least_squares
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Known positions of base stations
# base_stations = np.array([
#     [0, 0, 0],       # Base Station 1
#     [0, 4.5, 0],     # Base Station 2
#     [4.5, 0, 0],     # Base Station 3
#     [4.5, 4.5, 0]    # Base Station 4
# ])

# # TDOA measurements in meters
# tdoa_measurements = np.array([
#     1.0274027996115447,   # TDOA between P1 and P2
#     0.39525416704020455,  # TDOA between P1 and P3
#     -0.5296863659009495   # TDOA between P1 and P4
# ])

# def objective_function(position, base_stations, tdoa_measurements):
#     P1 = base_stations[0]
#     differences = []
    
#     for i, (P2, tdoa) in enumerate(zip(base_stations[1:], tdoa_measurements)):
#         dist1 = np.linalg.norm(position - P1)
#         dist2 = np.linalg.norm(position - P2)
#         differences.append((dist1 - dist2) - tdoa)
    
#     return differences

# # Initial guess for the tag's position
# initial_guess = np.array([2, 2, 0])

# # Perform least squares optimization
# result = least_squares(objective_function, initial_guess, args=(base_stations, tdoa_measurements))

# # Extract the optimized position
# optimized_position = result.x
# print("Optimized Position:", optimized_position)

# # Create a grid of points for visualization
# x = np.linspace(-10, 10, 200)
# y = np.linspace(-10, 10, 200)
# z = np.linspace(-10, 10, 200)
# X, Y, Z = np.meshgrid(x, y, z)

# # Calculate distances to the base stations
# dist1 = np.sqrt((X - base_stations[0][0])**2 + (Y - base_stations[0][1])**2 + (Z - base_stations[0][2])**2)
# dist2 = np.sqrt((X - base_stations[1][0])**2 + (Y - base_stations[1][1])**2 + (Z - base_stations[1][2])**2)
# dist3 = np.sqrt((X - base_stations[2][0])**2 + (Y - base_stations[2][1])**2 + (Z - base_stations[2][2])**2)
# dist4 = np.sqrt((X - base_stations[3][0])**2 + (Y - base_stations[3][1])**2 + (Z - base_stations[3][2])**2)

# # Calculate the difference in distances
# delta_dist1 = dist1 - dist2
# delta_dist2 = dist1 - dist3
# delta_dist3 = dist1 - dist4

# # Find points where the difference in distances matches the TDOA measurements
# tolerance = 1e-3
# mask1 = np.abs(delta_dist1 - tdoa_measurements[0]) < tolerance
# mask2 = np.abs(delta_dist2 - tdoa_measurements[1]) < tolerance
# mask3 = np.abs(delta_dist3 - tdoa_measurements[2]) < tolerance

# # Plot the hyperboloids and base stations
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(X[mask1], Y[mask1], Z[mask1], s=1, label='Hyperboloid 1')
# ax.scatter(X[mask2], Y[mask2], Z[mask2], s=1, label='Hyperboloid 2', alpha=0.5)
# ax.scatter(X[mask3], Y[mask3], Z[mask3], s=1, label='Hyperboloid 3', alpha=0.5)

# # Plot the base stations
# ax.scatter(base_stations[:, 0], base_stations[:, 1], base_stations[:, 2], color=['red', 'blue', 'green', 'orange'], s=100, label=['Base Station 1 (0, 0, 0)', 'Base Station 2 (0, 4.5, 0)', 'Base Station 3 (4.5, 0, 0)', 'Base Station 4 (4.5, 4.5, 0)'])

# # Plot the true position
# ax.scatter(true_position[0], true_position[1], true_position[2], color='purple', s=100, label='True Position (2, 3, 0)')

# # Plot the optimized position
# ax.scatter(optimized_position[0], optimized_position[1], optimized_position[2], color='yellow', s=100, label='Optimized Position')

# # Add labels and legend
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.legend()

# plt.show()
