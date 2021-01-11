import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import math


def normalize(v):
    norm = np.linalg.norm(v,axis=0) + 0.00001
    return v / norm.reshape(1, v.shape[1])

def curvature(waypoints):
    '''
    ##### TODO #####
    Curvature as  the sum of the normalized dot product between the way elements
    Implement second term of the smoothin objective.

    args: 
        waypoints [2, num_waypoints] !!!!!
    '''

    curvature = 0
    for i in range(1, waypoints.shape[1] - 1):
                
        i_point = np.array(waypoints[:,i]).reshape(2,-1)
        j_point = np.array(waypoints[:,i+1]).reshape(2,-1)
        k_point = np.array(waypoints[:,i-1]).reshape(2,-1)
        
        value = np.dot(normalize(j_point - i_point).T, normalize(i_point - k_point)).flatten()
        curvature = curvature + value

    return curvature


def smoothing_objective(waypoints, waypoints_center, weight_curvature=40):
    '''
    Objective for path smoothing

    args:
        waypoints [2 * num_waypoints] !!!!!
        waypoints_center [2 * num_waypoints] !!!!!
        weight_curvature (default=40)
    '''
    waypoints = waypoints.reshape(2, -1)
    # mean least square error between waypoint and way point center
    ls_tocenter = np.mean((waypoints_center - waypoints)**2)

    # derive curvature
    curv = curvature(waypoints.reshape(2,-1))

    return (-1 * weight_curvature * curv) + ls_tocenter


def waypoint_prediction(roadside1_spline, roadside2_spline, num_waypoints=6, way_type = "smooth"):
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
    if way_type == "center":
        ##### TODO #####
     
        # create spline arguments

        t = np.linspace(0, 1, 6)
        way_points = np.zeros((2, num_waypoints))

        # derive roadside points from spline

        # evaluates the spline for equidistant 6 points along lane boundaries?
        lane_boundary1_points_points = np.array(splev(t, roadside1_spline))
        lane_boundary2_points_points = np.array(splev(t, roadside2_spline))

        # derive center between corresponding roadside points
        minPoints = min(
            lane_boundary1_points_points.shape[1], lane_boundary2_points_points.shape[1]
        )
        for i in range(minPoints):
            midPoint = np.array(
                (lane_boundary2_points_points[:, i] +
                 lane_boundary1_points_points[:, i])/2
            )
            way_points[:,i] = midPoint

        # output way_points with shape(2 x Num_waypoints)
        return way_points
    
    elif way_type == "smooth":
        ##### TODO #####

        # create spline arguments

        # derive roadside points from spline

        # derive center between corresponding roadside points
        
        way_points_center = waypoint_prediction(
            roadside1_spline, roadside2_spline, way_type="center")
        
        # optimization
        way_points = minimize(smoothing_objective, 
                      (way_points_center), 
                      args=way_points_center)["x"]

        return way_points.reshape(2,-1)


def target_speed_prediction(waypoints, num_waypoints_used=6,
                            max_speed=60, exp_constant=6, offset_speed=30):
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
#    print(max_speed)
    K = exp_constant
        
    value = -K * abs(num_waypoints_used - 2 - curvature(waypoints))
    
    target_speed = ((max_speed - offset_speed) * math.exp(value)) + offset_speed
    # if target_speed > max_speed: target_speed = max_speed

    return target_speed
