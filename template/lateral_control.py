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

        # derive orientation error as the angle of the first path segment to the car orientation
        first = waypoints[:, 1] - np.array([48, 0])
        second = waypoints[:, 0] - np.array([48, 0])

        third = waypoints[:, 2] - np.array([48, 0])

        second_minus_first = first - second

        third_minus_second = third - second

        tan_theta_1 = second_minus_first[0] / second_minus_first[1]
        orientation_error_1 = np.arctan(tan_theta_1)

        tan_theta_2 = third_minus_second[0] / third_minus_second[1]
        orientation_error_2 = np.arctan(tan_theta_2)

        orientation_error = orientation_error_1 * 0.7 + orientation_error_2 * 0.3
        # derive cross track error as distance between desired waypoint at spline parameter equal zero ot the car position
        # (48,0 is the position of the car). Since we only want lateral distance,
        # the cross track error is the difference between the first waypoint's x dimention & 48.
        cross_track_error = (waypoints[0, 0] - 48) * 0.8

        # derive stanley control law
        # prevent division by zero by adding as small epsilon
        control_law = orientation_error + np.arctan(self.gain_constant * cross_track_error / (speed + 0.00001) )

        # derive damping term
        steering_angle = control_law - (self.damping_constant * (control_law - self.previous_steering_angle))

        self.previous_steering_angle = steering_angle
        # clip to the maximum stering angle (0.4) and rescale the steering action space
        ret = np.clip(steering_angle, -0.4, 0.4) /0.8
        return ret






