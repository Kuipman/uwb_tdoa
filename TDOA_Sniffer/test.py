import numpy as np

# Sample TDOA data
tdoa_data = np.array([2.5, 3.6, 2.8, 3.9, 4.2, 5.1, 3.3, 2.7, 3.1, 2.9])

# Min-Max Normalization
def min_max_normalize(data):
    min_val = np.min(data)
    max_val = np.max(data)
    normalized = (data - min_val) / (max_val - min_val)
    return normalized

# Z-Score Normalization
def z_score_normalize(data):
    mean_val = np.mean(data)
    std_dev = np.std(data)
    normalized = (data - mean_val) / std_dev
    return normalized

# Applying normalization
normalized_min_max = min_max_normalize(tdoa_data)
normalized_z_score = z_score_normalize(tdoa_data)

# Print normalized data
print("Min-Max Normalized Data:", normalized_min_max)
print("Z-Score Normalized Data:", normalized_z_score)
