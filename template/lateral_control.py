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


    def __init__(self, gain_constant=5, damping_constant=0.1):
        #damping_constant = 0.6

        self.gain_constant = gain_constant
        self.damping_constant = damping_constant
        self.previous_steering_angle = 0


    def stanley(self, waypoints, speed):
        '''
        ##### TODO #####
        one step of the stanley controller with damping
        args:
            waypoints (np.array) [2, num_waypoints]
            speed (float)
        '''
#        waypoints = np.array([[4.74299617e+01, 4.74296727e+01, 4.75480819e+01, 4.76715591e+01,
#                               4.78097113e+01, 4.80111628e+01],[2.11004204e-02, 1.33729666e+01 ,2.69786568e+01, 4.02988814e+01,
#                                     5.36131275e+01, 6.69793708e+01]])

        #print(waypoints)
        A = np.reshape(waypoints[:,0],(2,-1))
        B = np.reshape(waypoints[:,1],(2,-1))
        C = B-A
        _A = np.reshape(np.array([0,1]),(2,-1))
        #print(_A.shape)
        D = np.dot(C.T,_A)
        D = D/np.linalg.norm(C)
        print(f"D {D}")
        #print(D.shape)
        ori_error = np.arccos(D).item()
        print(f'ORIENTATION {ori_error}')
        car_pos = np.reshape(np.array([48,0]),(2,-1))
        cross_track = (A[0]-car_pos[0]).item()
        sc = ori_error + np.arctan((self.gain_constant*cross_track)/(speed+0.1))
        print(f'Cross track {cross_track}')

        
        
        # derive orientation error as the angle of the first path segment to the car orientation

        # derive cross track error as distance between desired waypoint at spline parameter equal zero ot the car position

        # derive stanley control law
        # prevent division by zero by adding as small epsilon

        # derive damping term
        
        steering_angle = sc - self.damping_constant*(sc-self.previous_steering_angle)
        #print(f'SC {steering_angle}')
        #steering_angle = sc
        self.previous_steering_angle = steering_angle
        
        # clip to the maximum stering angle (0.4) and rescale the steering action space
        print(f'STEER {np.clip(steering_angle, -0.4, 0.4) / 0.4}')
        return np.clip(steering_angle, -0.4, 0.4) / 0.4






