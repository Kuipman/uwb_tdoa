import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Example parameters
d = 10  # distance between receivers
d_12 = 4  # distance difference derived from TDOA measurement

# Calculate a and b
a = d_12 / 2
b = np.sqrt(d**2 - d_12**2) / 2

# Generate y and z values
y = np.linspace(-5, 5, 100)
z = np.linspace(-5, 5, 100)
y, z = np.meshgrid(y, z)

# Calculate x' based on the recasting Equation (5)
x_pos = a * np.sqrt(1 + (y**2 / b**2) + (z**2 / b**2))
x_neg = -a * np.sqrt(1 + (y**2 / b**2) + (z**2 / b**2))

# Plotting the hyperboloid
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot_surface(x_pos, y, z, alpha=0.6, cmap='jet')
ax.plot_surface(x_neg, y, z, alpha=0.6, cmap='jet')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Hyperboloid of One Sheet from TDOA')

plt.show()
