import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from scipy.interpolate import splprep, splev
from scipy.optimize import minimize
import time


class LateralController:
    '''
    Lateral control using the Stanley controller

    functions:
        stanley 

    init:
        gain_constant (default=5)
        damping_constant (default=0.5)
    '''


    def __init__(self, gain_constant=5, damping_constant=0.6):

        self.gain_constant = 2
        self.damping_constant = 0.05
        self.previous_steering_angle = 0


    def stanley(self, waypoints, speed):
        '''
        ##### TODO #####
        one step of the stanley controller with damping
        args:
            waypoints (np.array) [2, num_waypoints]
            speed (float)
        '''
        # derive orientation error as the angle of the first path segment to the car orientation
        A = np.reshape(waypoints[:,0],(2,-1))
        B = np.reshape(waypoints[:,1],(2,-1))
        C = B-A
        _A = np.reshape(np.array([0,1]),(2,-1))
        #print(_A.shape)
        D = np.dot(C.T,_A)
        D = D/np.linalg.norm(C)
        # print(f"D {D}")
        #print(D.shape)
        ori_error = np.arccos(D).item()

        # derive cross track error as distance between desired waypoint at 
        #   spline parameter equal zero ot the car position
        car_pos = np.reshape(np.array([48,0]),(2,-1))
        cross_track = (A[0]-car_pos[0]).item()

        # derive stanley control law
        # prevent division by zero by adding as small epsilon
        epsilon = 0.0001
        sc = ori_error + np.arctan((self.gain_constant*cross_track)/(speed+epsilon))

        # derive damping term
        
        steering_angle = sc - self.damping_constant*(sc-self.previous_steering_angle)
        self.previous_steering_angle = steering_angle
        # clip to the maximum stering angle (0.4) and rescale the steering action space
        return np.clip(steering_angle, -0.4, 0.4) / 0.4






