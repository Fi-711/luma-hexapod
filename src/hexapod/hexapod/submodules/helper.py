import numpy as np
import time
import math


# def wait_for_seconds():
#     start_time = time.time()
#     elapsed_time = time.time() - start_time

#     while elapsed_time <= start_time:


def servo_centre(array):
    """
    Maestro uses 0.25 tick/ usecond. Return result to quarter seconds.
    """
    return sum(array) / len(array) * 4


def bernstein(t, n, i):
    """
    Bernstein polynomial function for a given parameter t, degree n, and index i.
    """
    return np.math.comb(n, i) * (t**i) * ((1-t)**(n-i))


def bezier_curve(control_points, num_samples):
    """
    Evaluate a Bezier curve using Bernstein's algorithm.

    control_points (list of tuples): List of control points as (x, y, z) tuples.
    num_samples (int): Number of samples (time values) to evaluate the curve.

    Returns a list of points on the Bezier curve as (x, y, z) tuples.
    """
    n = len(control_points) - 1  # Degree of the Bezier curve
    curve_points = []

    for t in np.linspace(0, 1, num_samples):
        point = np.zeros(3)  # Initialize the point as (0, 0, 0)
        for i in range(n + 1):
            point += np.array(control_points[i]) * bernstein(t, n, i)
        curve_points.append(tuple(point))

    return curve_points


def bezier_interpolation(points, t):
    """
    Interpolate and move along the 3D Bézier curve
    """
    # Calculate the number of control points
    n = len(points)
    
    # Initialize the interpolated position
    pos = [0, 0, 0]
    
    # Iterate over the control points
    for i in range(n):
        # Calculate the Bernstein polynomial
        B = ((1 - t)**(n - 1 - i)) * (t**i)
        
        # Update the interpolated position with the weighted control point
        pos[0] += points[i][0] * B
        pos[1] += points[i][1] * B
        pos[2] += points[i][2] * B
        
    return pos


def linear_interpolate_line(start_point, end_point, num_points):
    """
    Interpolates 3D points along a straight line between start_point and end_point.

    Args:
        start_point (tuple or list): The starting point of the line in 3D space, as (x, y, z).
        end_point (tuple or list): The ending point of the line in 3D space, as (x, y, z).
        num_points (int): The number of points to interpolate along the line, including the start and end points.

    Returns:
        list: A list of 3D points along the straight line, including the start and end points.
    """
    # Convert start_point and end_point to numpy arrays for easier calculations
    start_point = np.array(start_point)
    end_point = np.array(end_point)

    # Calculate the step size for interpolation
    step = (end_point - start_point) / (num_points - 1)

    # Interpolate points along the line
    points = [start_point + i * step for i in range(num_points)]

    return points


def linear_interpolation(points, t):
    """
    Linear interpolation function for 3D points.

    points (list): List of 3D points in the form [(x0, y0, z0), (x1, y1, z1), ..., (xn, yn, zn)]
    t (float): Parameter value between 0 and 1 for interpolation

    Returns Interpolated 3D point as a tuple (x, y, z)
    """
    # Get the number of control points
    n = len(points)
    
    # Ensure that t is within the valid range [0, 1]
    t = max(0.0, min(t, 1.0))
    
    # Calculate the index of the first control point
    index = int(t * (n - 1))
    
    # Calculate the fraction for interpolation
    frac = t * (n - 1) - index
    
    # Get the two control points to interpolate between
    p0 = points[index]
    p1 = points[index + 1]
    
    # Perform linear interpolation for each coordinate separately
    x = p0[0] + frac * (p1[0] - p0[0])
    y = p0[1] + frac * (p1[1] - p0[1])
    z = p0[2] + frac * (p1[2] - p0[2])
    
    # Return the interpolated 3D point as a tuple
    return x, y, z


def are_collinear(p1, p2, p3):
    """
    Function to check if three points are collinear
    """
    return (p1[0] - p2[0]) * (p1[1] - p3[1]) == (p1[1] - p2[1]) * (p1[0] - p3[0])


def reduce_linear_points(points):
    """
    Reduce control points on a straight line by linear interpolation
    """
    # Start with the first point
    reduced_points = [points[0]]  
    
    for i in range(1, len(points) - 1):
        if are_collinear(points[i - 1], points[i], points[i + 1]):
            # If the three points are collinear, skip the middle point
            continue
        else:
            reduced_points.append(points[i])  # Add non-collinear points to reduced points
    reduced_points.append(points[-1])  # Add the last point

    return reduced_points


# Function to calculate cubic Bezier curve point at parameter t
def bezier_curve_point(t, P0, P1, P2, P3):
    u = 1 - t
    uu = u * u
    uuu = uu * u
    tt = t * t
    ttt = tt * t
    # return (uuu * P0) + (3 * uu * t * P1) + (3 * u * tt * P2) + (ttt * P3)
    return (uuu * P0) + (3 * uu * t * (P1 - P0)) + (3 * u * tt * (P2 - P1)) + (ttt * (P3 - P2)) + P0


# Define the 3D Bézier curve equation
def bezier_curve_3d(t, P0, P1, P2, P3):
    return (1 - t) ** 3 * P0 + 3 * (1 - t) ** 2 * t * P1 + 3 * (1 - t) * t ** 2 * P2 + t ** 3 * P3


# Check if a point lies on the 3D Bézier curve
def is_point_on_bezier_curve_3d(point, control_points, tolerance=1e-6):
    P0 = np.array(control_points[0])
    P1 = np.array(control_points[1])
    P2 = np.array(control_points[2])
    P3 = np.array(control_points[3])

    # Iterate over a range of t values from 0 to 1
    for t in np.linspace(0, 1, num=1000):
        # Calculate the point on the Bézier curve for the current t
        curve_point = bezier_curve_3d(t, P0, P1, P2, P3)
        # Calculate the distance between the current point and the curve point
        distance = np.linalg.norm(point - curve_point)
        # If the distance is within the tolerance, the point is considered to be on the curve
        if distance < tolerance:
            return True
    return False


def sine_ease_in_out(t):
    """
    Sine ease-in and ease-out function.
    
    Parameters:
        t (float): Input time value, ranging from 0 to 1.
    
    Returns:
        float: Output value adjusted by sine ease-in and ease-out.
    """
    return 0.5 * (1 - math.cos(t * math.pi))


def euclidean_distance(p1, p2):
    """
    Calculates the Euclidean distance between 2 3d points.
    """
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)


def reverse_control_point(control_points):
    """
    Reverse control points so leg now moves backwards
    """
    return control_points[:3][::-1] + control_points[3:]


def translate_point(point, translation_vector):
    """
    translates a point in by the translation vector
    """
    # convert to np arrays so can use matrix calculations
    point = np.array(point)
    translation_vector = np.array(translation_vector)
    
    # make translkation matrix
    translation_matrix = np.eye(3)
    translation_matrix[:, 2] = translation_vector

    return translation_matrix @ point


def rotate_point(point, angle, plane="y"):
    """
    translates a point in by the translation vector
    """
    # convert to np arrays so can use matrix calculations
    point = np.array(point)
    theta = np.deg2rad(angle)
    
    # make translkation matrix
    rotation_matrices = { 
        "x": np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]]),
        "y": np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]]),
        "z": np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
    }

    return rotation_matrices[plane] @ point






