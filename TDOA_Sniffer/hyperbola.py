import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import welch

# Function to generate a conventional AM/FM radio signal
def generate_am_fm_signal(fs, duration):
    t = np.linspace(0, duration, int(fs * duration), endpoint=False)
    carrier_freq = 1e6  # 1 MHz
    modulating_freq = 1e3  # 1 kHz
    modulating_signal = np.cos(2 * np.pi * modulating_freq * t)
    carrier_signal = np.cos(2 * np.pi * carrier_freq * t)
    am_signal = (1 + modulating_signal) * carrier_signal
    fm_signal = np.cos(2 * np.pi * carrier_freq * t + 2 * np.pi * 100 * np.sin(2 * np.pi * modulating_freq * t))
    return t, am_signal, fm_signal

# Function to generate an Ultra-Wideband (UWB) signal
def generate_uwb_signal(fs, duration):
    t = np.linspace(0, duration, int(fs * duration), endpoint=False)
    pulse_width = 1e-9  # 1 ns
    pulse_rate = 1e6  # 1 MHz
    uwb_signal = np.zeros_like(t)
    pulse_times = np.arange(0, duration, 1 / pulse_rate)
    for pt in pulse_times:
        idx = (np.abs(t - pt)).argmin()
        uwb_signal[idx:idx + int(pulse_width * fs)] = 1
    return t, uwb_signal

# Parameters
fs = 10e6  # Sampling frequency 10 MHz
duration = 0.01  # 10 ms

# Generate signals
t_am_fm, am_signal, fm_signal = generate_am_fm_signal(fs, duration)
t_uwb, uwb_signal = generate_uwb_signal(fs, duration)

# Compute power spectral density using Welch's method
frequencies_am, psd_am = welch(am_signal, fs, nperseg=1024)
frequencies_fm, psd_fm = welch(fm_signal, fs, nperseg=1024)
frequencies_uwb, psd_uwb = welch(uwb_signal, fs, nperseg=1024)

# Plot the results
plt.figure(figsize=(14, 8))

plt.subplot(3, 1, 1)
plt.semilogy(frequencies_am, psd_am)
plt.title('Power Spectral Density of AM Signal')
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD (dB/Hz)')

plt.subplot(3, 1, 2)
plt.semilogy(frequencies_fm, psd_fm)
plt.title('Power Spectral Density of FM Signal')
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD (dB/Hz)')

plt.subplot(3, 1, 3)
plt.semilogy(frequencies_uwb, psd_uwb)
plt.title('Power Spectral Density of UWB Signal')
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD (dB/Hz)')

plt.tight_layout()
plt.show()






# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Known positions of base stations
# P1 = np.array([0, 0, 0])
# P2 = np.array([0, 4.5, 0])
# P3 = np.array([4.5, 0, 0])
# P4 = np.array([4.5, 4.5, 0])

# # TDOA measurements in meters
# delta_d1 = 1.0274027996115447  # TDOA between P1 and P2
# delta_d2 = -0.39525416704020455  # TDOA between P1 and P3
# delta_d3 = 0.5296863659009495  # TDOA between P1 and P4

# # True position of the tag
# true_position = np.array([2, 3, 0])

# # Create a grid of points
# x = np.linspace(-10, 10, 200)
# y = np.linspace(-10, 10, 200)
# z = np.linspace(-10, 10, 200)
# X, Y, Z = np.meshgrid(x, y, z)

# # Calculate distances to the base stations
# dist1 = np.sqrt((X - P1[0])**2 + (Y - P1[1])**2 + (Z - P1[2])**2)
# dist2 = np.sqrt((X - P2[0])**2 + (Y - P2[1])**2 + (Z - P2[2])**2)
# dist3 = np.sqrt((X - P3[0])**2 + (Y - P3[1])**2 + (Z - P3[2])**2)
# dist4 = np.sqrt((X - P4[0])**2 + (Y - P4[1])**2 + (Z - P4[2])**2)

# # Calculate the difference in distances
# delta_dist1 = dist1 - dist2
# delta_dist2 = dist1 - dist3
# delta_dist3 = dist1 - dist4

# # Find points where the difference in distances matches the TDOA measurements
# tolerance = 1e-3
# mask1 = np.abs(delta_dist1 - delta_d1) < tolerance
# mask2 = np.abs(delta_dist2 - delta_d2) < tolerance
# mask3 = np.abs(delta_dist3 - delta_d3) < tolerance

# # Plot the hyperboloids and base stations
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(X[mask1], Y[mask1], Z[mask1], s=1, label='Hyperboloid 1')
# ax.scatter(X[mask2], Y[mask2], Z[mask2], s=1, label='Hyperboloid 2', alpha=0.5)
# ax.scatter(X[mask3], Y[mask3], Z[mask3], s=1, label='Hyperboloid 3', alpha=0.5)

# # Plot the base stations
# ax.scatter(P1[0], P1[1], P1[2], color='red', s=100, label='Base Station 1 (0, 0, 0)')
# ax.scatter(P2[0], P2[1], P2[2], color='blue', s=100, label='Base Station 2 (0, 4.5, 0)')
# ax.scatter(P3[0], P3[1], P3[2], color='green', s=100, label='Base Station 3 (4.5, 0, 0)')
# ax.scatter(P4[0], P4[1], P4[2], color='orange', s=100, label='Base Station 4 (4.5, 4.5, 0)')

# # Plot the true position
# ax.scatter(true_position[0], true_position[1], true_position[2], color='purple', s=100, label='True Position (2, 3, 0)')

# # Add labels and legend
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.legend()

# plt.show()







# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Known positions of base stations
# P1 = np.array([0, 0, 0])
# P2 = np.array([0, 4.5, 0])
# c = 3e8  # Speed of light in meters per second (example, adjust if using different signals)
# delta_t = 1.0274027996115447 / c  # Convert distance difference to time difference

# # Create a grid of points
# x = np.linspace(-10, 10, 100)
# y = np.linspace(-10, 10, 100)
# z = np.linspace(-10, 10, 100)
# X, Y, Z = np.meshgrid(x, y, z)

# # Calculate distances to the base stations
# dist1 = np.sqrt((X - P1[0])**2 + (Y - P1[1])**2 + (Z - P1[2])**2)
# dist2 = np.sqrt((X - P2[0])**2 + (Y - P2[1])**2 + (Z - P2[2])**2)

# # Calculate the difference in distances
# delta_d = dist1 - dist2

# # Find points where the difference in distances matches 1.0274027996115447 meters
# tolerance = 1e-3
# mask = np.abs(delta_d - 1.0274027996115447) < tolerance

# # Plot the hyperboloid and base stations
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(X[mask], Y[mask], Z[mask], s=1, label='Hyperboloid')

# # Plot the base stations
# ax.scatter(P1[0], P1[1], P1[2], color='red', s=100, label='Base Station 1 (0, 0, 0)')
# ax.scatter(P2[0], P2[1], P2[2], color='blue', s=100, label='Base Station 2 (0, 4.5, 0)')

# # Add labels and legend
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.legend()

# plt.show()




# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # Example parameters
# d = 10  # distance between receivers
# d_12 = 4  # distance difference derived from TDOA measurement

# # Calculate a and b
# a = d_12 / 2
# b = np.sqrt(d**2 - d_12**2) / 2

# # Generate y and z values
# y = np.linspace(-5, 5, 100)
# z = np.linspace(-5, 5, 100)
# y, z = np.meshgrid(y, z)

# # Calculate x' based on the recasting Equation (5)
# x_pos = a * np.sqrt(1 + (y**2 / b**2) + (z**2 / b**2))
# x_neg = -a * np.sqrt(1 + (y**2 / b**2) + (z**2 / b**2))

# # Plotting the hyperboloid
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# ax.plot_surface(x_pos, y, z, alpha=0.6, cmap='jet')
# ax.plot_surface(x_neg, y, z, alpha=0.6, cmap='jet')

# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('Hyperboloid of One Sheet from TDOA')

# plt.show()
