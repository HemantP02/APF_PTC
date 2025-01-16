import matplotlib.pyplot as plt

# Read data from the file
data = []
with open('simulationData_bicycle.txt', 'r') as file:
    for line in file:
        data.append([float(x) for x in line.strip().split()])

# Separate the data into time, x, y, and theta
time = [row[0] for row in data]
x = [row[1] for row in data]
y = [row[2] for row in data]
theta = [row[3] for row in data]

# Extract starting and ending positions
start_x, start_y, _ = data[0][1:]
end_x, end_y, _ = data[-1][1:]

# Plot the trajectory, starting, and ending positions
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Trajectory')
plt.plot(start_x, start_y, 'bo', label='Starting Position')
plt.plot(end_x, end_y, 'ro', label='Ending Position')
plt.title('Robot Trajectory with Starting and Ending Positions')
plt.xlabel('X-coordinate')
plt.ylabel('Y-coordinate')
plt.grid(True)
plt.legend()
plt.show()
