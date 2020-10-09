import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time

NUM_WAYPOINTS = 6


def normalize(v):
    norm = np.linalg.norm(v, axis=0) + 0.00001
    return v / norm.reshape(1, v.shape[1])

def curvature(waypoints):
    '''
    ##### TODO #####
    Curvature as  the sum of the normalized dot product between the way elements
    Implement second term of the smoothin objective.

    args: 
        waypoints [2, num_wa1ypoints] !!!!!
    '''
    curvature = 0
    for i in range(1, waypoints.shape[1]-1):
        nplus1 = waypoints[:, i + 1]
        n = waypoints[:, i]
        nminus1 = waypoints[:, i - 1]
        curvature = curvature + (np.dot(nplus1 - n,n - nminus1) / np.dot(abs(nplus1 - n), abs(n - nminus1)))
    return curvature


def smoothing_objective(waypoints, waypoints_center, weight_curvature=40):
    '''
    Objective for path smoothing

    args:
        waypoints [2 * num_waypoints] !!!!!
        waypoints_center [2 * num_waypoints] !!!!!
        weight_curvature (default=40)
    '''
    # mean least square error between waypoint and way point center
    ls_tocenter = np.mean((waypoints_center - waypoints.reshape(2, -1))**2)

    # derive curvature
    curv = curvature(waypoints.reshape(2, -1))

    return -1 * weight_curvature * curv + ls_tocenter


def waypoint_prediction(roadside1_spline, roadside2_spline, num_waypoints=NUM_WAYPOINTS, way_type = "smooth"):
    '''
    ##### TODO #####
    Predict waypoint via two different methods:
    - center
    - smooth 

    args:
        roadside1_spline
        roadside2_spline
        num_waypoints (default=6)
        parameter_bound_waypoints (default=1)
        waytype (default="smoothed")
    '''

    t = np.linspace(0, 1, NUM_WAYPOINTS)

    lane_boundary1_points_points = np.array(splev(t, roadside1_spline))
    lane_boundary2_points_points = np.array(splev(t, roadside2_spline))

    way_points_center = np.zeros((2, num_waypoints))

    for i in range(lane_boundary1_points_points.shape[1]):
        mid_offset = np.array((lane_boundary2_points_points[:, i] - lane_boundary1_points_points[:, i]) / 2)
        mid = lane_boundary1_points_points[:, i] + mid_offset
        way_points_center[:, i] = mid

    if way_type == "center":

        return way_points_center
    
    elif way_type == "smooth":

        way_points = minimize(smoothing_objective, way_points_center, args=way_points_center)["x"]
        return way_points.reshape(2, -1)


def target_speed_prediction(waypoints, num_waypoints_used=NUM_WAYPOINTS,
                            max_speed=60, exp_constant=4.5, offset_speed=30):
    '''
    ##### TODO #####
    Predict target speed given waypoints
    Implement the function using curvature()

    args:
        waypoints [2,num_waypoints]
        num_waypoints_used (default=5)
        max_speed (default=60)
        exp_constant (default=4.5)
        offset_speed (default=30)
    
    output:
        target_speed (float)
    '''

    target_speed = ((max_speed - offset_speed) * np.exp(-exp_constant * (num_waypoints_used - 2 - curvature(waypoints)))) + offset_speed
    return target_speed