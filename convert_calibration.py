import numpy as np

# Read the camera matrix and distortion vector from the numpy files
camera_matrix = np.load("RealSense_cam_matrix.npy")
distortion_vector = np.load("RealSense_cam_distor_vec.npy")

print(camera_matrix)
print(distortion_vector)

# Save the camera matrix and distortion vector as CSV files
np.savetxt("camera_matrix.csv", camera_matrix, delimiter=",")
np.savetxt("distortion_vector.csv", distortion_vector, delimiter=",")