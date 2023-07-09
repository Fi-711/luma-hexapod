import numpy as np
import matplotlib.pyplot as plt


def bernstein(t, n, i):
    """
    Bernstein polynomial function for a given parameter t, degree n, and index i.
    """
    return np.math.comb(n, i) * (t**i) * ((1-t)**(n-i))

def evaluate_bezier_curve(control_points, num_samples):
    """
    Evaluate a Bezier curve using Bernstein's algorithm.

    Args:
        control_points (list of tuples): List of control points as (x, y, z) tuples.
        num_samples (int): Number of samples (time values) to evaluate the curve.

    Returns:
        list of tuples: List of points on the Bezier curve as (x, y, z) tuples.
    """
    n = len(control_points) - 1  # Degree of the Bezier curve
    curve_points = []

    for t in np.linspace(0, 1, num_samples):
        point = np.zeros(3)  # Initialize the point as (0, 0, 0)
        for i in range(n + 1):
            point += np.array(control_points[i]) * bernstein(t, n, i)
        curve_points.append(tuple(point))

    return curve_points

# Example usage
# control_points = [(148, -6, -70), (148, 10, 10), (148, 25, -70)]
control_points = [(104, -104, -70), (110, -84, 0), (120, -64, 0), (130, -44, -70)]
num_samples = 40  # Number of samples to evaluate the curve
curve_points = evaluate_bezier_curve(control_points, num_samples)

# Extract x, y, z coordinates from the curve points
x = [point[0] for point in curve_points]
y = [point[1] for point in curve_points]
z = [point[2] for point in curve_points]

# Plot the Bezier curve in 3D space
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()