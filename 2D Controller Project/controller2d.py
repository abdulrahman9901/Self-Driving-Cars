#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 'v_previous', default value to be 0
            self.vars.create_var('v_previous', 0.0)

            Example: Setting 'v_previous' to be 1.0
            self.vars.v_previous = 1.0

            Example: Accessing the value from 'v_previous' to be used
            throttle_output = 0.5 * self.vars.v_previous
        """
        self.vars.create_var('ki', 0.0)
        self.vars.create_var('kp', 0.0)
        self.vars.create_var('kd', 0.0)
        self.vars.create_var('v_previous', 0.0)
        self.vars.create_var('e_previous', 0.0)
        self.vars.create_var('I_previous', 0.0)
        self.vars.create_var('D_previous', 0.0)
        self.vars.create_var('sampling_time', 0.0)
        self.vars.create_var('t_previous', 0.0)
        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """
            #kp=0.2
            #ki=0.3
            #kd=0.001

            #kp=0.15
            #ki=0.6
            #kd=0.015 #0.0001 looks well

            kp=0.1
            ki=0.1
            kd=0.01

            self.vars.sampling_time = t - self.vars.t_previous
            print('delta_t = ', self.vars.sampling_time)
            # I going to use PID controller in longitudinal control
            e = v_desired - v
            P = kp * e
            I = ((ki * self.vars.sampling_time / 2) * (e + self.vars.e_previous)) + self.vars.I_previous
            D = ((2 * kd / self.vars.sampling_time) * (e - self.vars.e_previous)) + self.vars.D_previous

            #v = (P + I + D) * self.vars.sampling_time + v

            print("speed at time " + str(t) + " = " + str(v) +" desired "+str(v_desired))

            # print(out_list[0])

            self.vars.D_previous = D
            self.vars.I_previous = I
            self.vars.e_previous = e

            Acc=P+I+D

            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.

            if Acc > 0:
                throttle_output = Acc
                brake_output    = 0
            else:
                throttle_output = 0
                brake_output = -Acc

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_previous like a "global variable".
            """

            ## Reference
            # https://math.stackexchange.com/questions/3289928/confused-about-dot-product-in-cross-track-error-calculation

            k=2.55
            ks=1

            if len(waypoints) > 5:
                desired_point = waypoints[-5]
            else:
                desired_point = waypoints[-1]

            yaw_path = np.arctan2(waypoints[-1][1] - waypoints[0][1],
                                  waypoints[-1][0] - waypoints[0][0])

            psi = (yaw_path - yaw + self._pi) % self._2pi - self._pi

            # Search nearest point index

            dx = [x - point[0] for point in waypoints]

            dy = [y - point[1] for point in waypoints]

            d = [np.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]

            front_axle_vec = [-np.cos(yaw + np.pi / 2), -np.sin(yaw + np.pi / 2)]

            closest_error = min(d)

            target_idx = d.index(closest_error)

            error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

            delta = psi + np.arctan2(k*error_front_axle,ks+v)

            # Change the steer output with the lateral controller. 
            steer_output    = delta

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        self.vars.v_previous = v  # Store forward speed to be used in next step
        self.vars.t_previous = t
