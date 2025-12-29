import numpy as np

# Example data (e.g., TDOA measurements)
measurements = np.array([10.2, 10.1, 9.8, 10.0, 10.3, 9.9, 10.2, 10.1, 10.0, 10.1])

# Assuming true state is the mean of the measurements
true_state = np.mean(measurements)

# Calculate residuals
residuals = measurements - true_state

# Calculate variance (for scalar measurements, this is the covariance)
noise_variance = np.var(residuals, ddof=1)

print("Noise Variance:", noise_variance)

def noiseModel(anchorList, anchor_id, remote_id, newTdoa):
    
