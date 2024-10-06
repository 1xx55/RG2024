import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

# Initialize lists for X, Y, Z coordinates
x_data = []
y_data = []
z_data = []

# Read CSV file
with open('detection_results.csv', 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    next(plots)  # Skip the header row
    for row in plots:
        # Remove square brackets and split the string to get coordinates
        tag_position = row[2].strip('[]').split()
        # Convert the strings to floats and append to the lists
        x_data.append(float(tag_position[0]))
        y_data.append(float(tag_position[1]))
        z_data.append(float(tag_position[2]))

# Plot 3D trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_data, y_data, z_data, label='Tag 3D Trajectory', marker='o')

# Set axis labels
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.legend()

# Display the plot
# plt.show()
plt.savefig('3d_trajectory.png')  # Save the figure as a PNG image
