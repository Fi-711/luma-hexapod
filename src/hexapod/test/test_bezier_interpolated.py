"""
Plot the shape of the Bezier curve. Trial and error to get it to be the correct shape in 3d space.
"""

import numpy as np
import matplotlib.pyplot as plt

# Control points of the Bézier curve
points = np.array([(120, -6, -60), (250, 10, 100), (120, 25, -60)])

# Generate points along the curve for visualization
t = np.linspace(0, 1, 100)
x = np.zeros_like(t)
y = np.zeros_like(t)
z = np.zeros_like(t)

# Perform Bézier interpolation
for i in range(len(t)):
    pos = np.zeros(3)
    for j in range(len(points)):
        pos += points[j] * ((1 - t[i]) ** (len(points) - 1 - j)) * (t[i] ** j)
    x[i] = pos[0]
    y[i] = pos[1]
    z[i] = pos[2]

# Plot the resulting curve
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z)
ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='o')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()