import matplotlib.pyplot as plt

def calculate_velocity(positions, timestep):
    velocity = []
    
    for i in range(1, len(positions)):
        diff = positions[i] - positions[i - 1]
        vel = diff / timestep
        velocity.append(vel)
    
    return velocity

# Initialize lists to store data
timestamps = []
true_positions = []
estimated_positions = []
true_velocities = []
estimated_velocities = []

filename = "data/Q_01_100_R_1.txt"
# Read data from file
with open(filename, 'r') as file:
    # Skip the first line
    next(file)
    
    for line in file:
        data = line.strip().split(',')
        timestamp = float(data[0])
        true_position = [float(data[i]) for i in range(1, 4)]
        true_velocity = [float(data[i]) for i in range(4, 7)]
        estimated_position = [float(data[i]) for i in range(7, 10)]
        estimated_velocity = [float(data[i]) for i in range(10, 13)]
        
        timestamps.append(timestamp)
        true_positions.append(true_position)
        true_velocities.append(true_velocity)
        estimated_positions.append(estimated_position)
        estimated_velocities.append(estimated_velocity)

# Extract individual position components
true_x = [position[0] for position in true_positions]
true_y = [position[1] for position in true_positions]
true_z = [position[2] for position in true_positions]

estimated_x = [position[0] for position in estimated_positions]
estimated_y = [position[1] for position in estimated_positions]
estimated_z = [position[2] for position in estimated_positions]

# Extract individual velocity components
true_x_dot = [velocity[0] for velocity in true_velocities]
true_y_dot = [velocity[1] for velocity in true_velocities]
true_z_dot = [velocity[2] for velocity in true_velocities]

estimated_x_dot = [velocity[0] for velocity in estimated_velocities]
estimated_y_dot = [velocity[1] for velocity in estimated_velocities]
estimated_z_dot = [velocity[2] for velocity in estimated_velocities]

timestep = timestamps[1] - timestamps[0]

# Calculate differentiated velocities
differentiated_x_dot = calculate_velocity(estimated_x, timestep)
differentiated_y_dot = calculate_velocity(estimated_y, timestep)
differentiated_z_dot = calculate_velocity(estimated_z, timestep)

# Plotting
plt.figure(figsize=(10, 6))
plt.suptitle("Decreased variance on position, increased on velocity")
# Plot positions
plt.subplot(1, 2, 1)
#plt.plot(timestamps, true_x, label='True x')
#plt.plot(timestamps, true_y, label='True y')
plt.plot(timestamps, true_z, label='True z')
plt.plot(timestamps, estimated_z, label='Estimated z')
plt.title(f'Z-positions')
plt.xlabel('Time [s]')
plt.ylabel('Position [m]')
plt.legend()

# Plot velocities
plt.subplot(1, 2, 2)
#plt.plot(timestamps, true_x_dot, label='True x_dot')
#plt.plot(timestamps, true_y_dot, label='True y_dot')
plt.plot(timestamps, true_z_dot, label='True z_dot')
plt.plot(timestamps, estimated_z_dot, label='Estimated z_dot')
#plt.plot(timestamps[:-1], differentiated_z_dot, label='Differentiated z_dot')
plt.title(f'Z-velocities')
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()

# Plot differentiated positions
#plt.subplot(2, 2, 3)
#plt.plot(timestamps, estimated_x, label='Estimated x')
#plt.plot(timestamps, estimated_y, label='Estimated y')
#plt.plot(timestamps, estimated_z, label='Estimated z')
#plt.title('Estimated Positions')
#plt.xlabel('Time')
#plt.ylabel('Position')
#plt.legend()

# Plot estimated velocities
#plt.subplot(2, 2, 4)
#plt.plot(timestamps, estimated_x_dot, label='Estimated x_dot')
#plt.plot(timestamps, estimated_y_dot, label='Estimated y_dot')
#plt.plot(timestamps, estimated_z_dot, label='Estimated z_dot')
#plt.title('Estimated Velocities')
#plt.xlabel('Time')
#plt.ylabel('Velocity')
#plt.legend()

plt.tight_layout()
plt.show()