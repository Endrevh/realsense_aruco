import matplotlib.pyplot as plt

# Define the file path
file_paths = ['data/servo20.txt','data/servo40.txt','data/servo60.txt','data/servo80.txt','data/servo100.txt']

# Lists to store data
time_values = []
position_error_norms = [[],[],[],[],[]]
velocity_estimates = [[],[],[],[],[]]
file_lengths = []
# Read data from the file
for i in range(len(file_paths)):
    file_path = file_paths[i]
    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split(',')

            tcp_pose = [float(data[i]) for i in range(10, 13)]
            target_pose = [float(data[i]) for i in range(22, 25)]
            kalman_estimate = [float(data[i]) for i in range(1, 7)]
            
            position_error_norms[i].append(((tcp_pose[0] - target_pose[0])**2 +
                                    (tcp_pose[1] - target_pose[1])**2 +
                                    (tcp_pose[2] - target_pose[2])**2)**0.5)
            
            velocity_estimate = ((kalman_estimate[0]**2 + kalman_estimate[1]**2 + kalman_estimate[2]**2)**0.5)

            velocity_estimates[i].append(velocity_estimate)

    file_lengths.append(len(position_error_norms[i]))

# Find the minimum length of the files
min_length = min(file_lengths)
min_index = file_lengths.index(min_length)
min_file = file_paths[min_index]
with open(min_file, 'r') as file:
    for line in file:
        data = line.strip().split(',')
        time_values.append(float(data[0]))

# Truncate the lists to the minimum length
for i in range(len(file_paths)):
    position_error_norms[i] = position_error_norms[i][:min_length]
    velocity_estimates[i] = velocity_estimates[i][:min_length]

# Plotting
plt.figure(figsize=(10, 6))
#plt.plot(time_values, position_error_norms[0], label='Speed control')
#plt.plot(time_values, velocity_estimates[0], label='Position Estimate')
plt.plot(time_values, position_error_norms[0], label='Servo 20%')
plt.plot(time_values, position_error_norms[1], label='Servo 40%')
plt.plot(time_values, position_error_norms[2], label='Servo 60%')
plt.plot(time_values, position_error_norms[3], label='Servo 80%')
plt.plot(time_values, position_error_norms[4], color='purple', label='Servo 100%')

plt.xlabel('Time [s]')
plt.ylabel('Tracking error [m]')
plt.title('ServoL with different scaling factors')
plt.legend()
plt.grid(True)
plt.show()

