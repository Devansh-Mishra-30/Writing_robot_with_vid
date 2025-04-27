import math
import numpy as np
from parse_svg_path import parse_svg_for_paths, scale_coords_to_arena 


def circle_trajectory(t:float) -> np.array:
    """

    Returns desired position at time t (seconds) to make circular trajectory
    with end effector.
    """
    r = 0.15  # circle radius (meters)
    w = 0.50  # rad/s

    Y_des = np.array([r* math.cos(w*t) + 0.4, r*math.sin(w*t), 0.62,
                      -3.14, 0, 0]) # X, Y, Z, roll, pitch, yaw
    return Y_des


def point_trajectory(t) -> np.array:
    """
    Returns singular point trajectory
    """

    return np.array([0.3, 0.0 ,0.55,
                     -3.14, 0, 0])


def svg_trajectory(t, svg_path) -> np.array:
    """
    Returns svg trajectroy generated from svg file svg_path
    """

    dx = 0.4
    dy = 0.4
    x_min= 0.2
    z_min = 0.635

    svg_coords = parse_svg_for_paths(svg_path)
    scaled_coords = np.array(scale_coords_to_arena(svg_coords, dx=dx, dy=dy, x_min=x_min, z_min=z_min))

    i = math.floor(t / 0.1) # Calculate the index for every 0.1 seconds

    if i < len(scaled_coords):
    
        return np.concatenate((np.array(scaled_coords[i]), np.array([-3.14, 0, 0])))
    
    return np.concatenate((np.array(scaled_coords[-1]), np.array([-3.14, 0, 0])))