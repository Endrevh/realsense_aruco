import matplotlib.pyplot as plt

# Initialize lists to store data
timestamps = [0.033333*i for i in range(300)]
positions = []

# Read data from file
with open('data/measurement_noise_4.txt', 'r') as file:   
    for line in file:
        data = line.strip().split(',')
        timestamp = float(data[0])
        position = [float(data[i]) for i in range(0, 3)]
        positions.append(position)


x = [position[0] for position in positions]
y = [position[1] for position in positions]
z = [position[2] for position in positions]


# Plot true positions
plt.subplot(1, 1, 1)
plt.plot(timestamps, z, label='Cartesian position')
plt.title('Position')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()


plt.show()