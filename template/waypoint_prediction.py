import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time
import math

NUM_WAYPOINTS=6

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
    waypoints = waypoints.reshape(2,-1)
    
    
    # mean least square error between waypoint and way point center
    ls_tocenter = np.mean((waypoints_center - waypoints)**2)

    # derive curvature
    curv = curvature(waypoints.reshape(2,-1))

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

#    way_type = "center"
    
    if way_type == "center":
        ##### TODO #####
        
        t = np.linspace(0, 1, NUM_WAYPOINTS)

        
        lane_boundary1_points_points = np.array(splev(t, roadside1_spline))
        lane_boundary2_points_points = np.array(splev(t, roadside2_spline))
        
        way_points = np.zeros((2, num_waypoints))
        
        
        for i in range(lane_boundary1_points_points.shape[1]):
            mid_offset = np.array((lane_boundary2_points_points[:,i] - lane_boundary1_points_points[:,i])/2)
            mid = lane_boundary1_points_points[:,i] + mid_offset            
            way_points[:,i] = mid
        
        
     
        # create spline arguments

        # derive roadside points from spline

        # derive center between corresponding roadside points

        # output way_points with shape(2 x Num_waypoints)
        return way_points
    
    elif way_type == "smooth":
        ##### TODO #####

        # create spline arguments

        # derive roadside points from spline

        # derive center between corresponding roadside points
        
        t = np.linspace(0, 1, NUM_WAYPOINTS)

        
        lane_boundary1_points_points = np.array(splev(t, roadside1_spline))
        lane_boundary2_points_points = np.array(splev(t, roadside2_spline))
        
        way_points_center = np.zeros((2, num_waypoints))
        
        
        for i in range(lane_boundary1_points_points.shape[1]):
            mid_offset = np.array((lane_boundary2_points_points[:,i] - lane_boundary1_points_points[:,i])/2)
            mid = lane_boundary1_points_points[:,i] + mid_offset            
            way_points_center[:,i] = mid
        
        
        # optimization
        way_points = minimize(smoothing_objective, 
                      (way_points_center), 
                      args=way_points_center)["x"]

        return way_points.reshape(2,-1)


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
    K = 4.5 
        
    value = -K * np.abs(num_waypoints_used - 2 - curvature(waypoints))
    
    target_speed = ((max_speed - offset_speed) * math.exp(value)) + offset_speed
    
    return target_speed











