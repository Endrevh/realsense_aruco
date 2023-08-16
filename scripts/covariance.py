import numpy as np

def calculate_variance(data):
    mean = np.mean(data)
    squared_diff = [(x - mean)**2 for x in data]
    variance = np.mean(squared_diff)
    return variance

def main():
    file_path = "data/measurement_noise_4.txt"
    x_data = []
    y_data = []
    z_data = []

    with open(file_path, "r") as file:
        for line in file:
            x, y, z = map(float, line.strip().split(","))
            x_data.append(x)
            y_data.append(y)
            z_data.append(z)

    x_variance = calculate_variance(x_data)
    y_variance = calculate_variance(y_data)
    z_variance = calculate_variance(z_data)

    print(f"Filename: {file_path}")
    print(f"Variance of x-coordinate: {x_variance:.5f}")
    print(f"Variance of y-coordinate: {y_variance:.5f}")
    print(f"Variance of z-coordinate: {z_variance:.5f}")

if __name__ == "__main__":
    main()
