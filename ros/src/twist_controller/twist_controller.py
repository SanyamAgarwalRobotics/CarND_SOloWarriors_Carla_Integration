#!/usr/bin/env python

from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy 

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio,
        max_lat_accel, max_steer_angle, min_speed):
        
        #rospy.loginfo('my test - wheel_base:%s steer_ratio:%s max_lat_accel:%s max_steer_angle:%s', wheel_base, steer_ratio, max_lat_accel, max_steer_angle)
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        kp = 5.
        ki = 0.05
        kd = 0.0
        mn = 0.
        mx = 0.2
        
        #rospy.loginfo('my 2 test -self.min:%s self.max:%s A:%s C:%s',mn, mx)
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        tau = 0.5#12.#0.2#0.5#0.2
        ts = 0.02#1.#0.1#0.02#0.1
        self.low_pass_filter = LowPassFilter(tau, ts)
        
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time = rospy.get_time()#rospy.Time.now()
        
        
    def control(self, current_velocity, linear_velocity, angular_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if abs(linear_velocity) < 0.5:
            rospy.loginfo('my 5 test -linear_velocity:%s', linear_velocity)
            self.throttle_controller.reset()
            
        if not dbw_enabled:
            rospy.loginfo('my test -dbw_enabled:%s', dbw_enabled)
            self.throttle_controller.reset()
            return 0., 0., 0.
            
        #rospy.loginfo('my test control 1')
        time = rospy.get_time()#rospy.Time().now()
        #rospy.loginfo('my test control 1r')
        #sample_t = (time - self.last_time).nsecs / 1e9
        sample_t = (time - self.last_time)
        self.last_time = time
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        
        rospy.loginfo('my 2 test -S:%s CS:%s', steer, corrective_steer)
        rospy.loginfo('my 2 test -L:%s A:%s C:%s', linear_velocity, angular_velocity, current_velocity)
              
        cte = linear_velocity - current_velocity
        
        #rospy.loginfo('my test control 2')
        acceleration = self.throttle_controller.step(cte, sample_t)
        
        acceleration = self.low_pass_filter.filt(acceleration)
        #rospy.loginfo('my 3 test -acceleration:%s', acceleration)
        brake = 0.
        throttle = acceleration
        
        #if linear_velocity < 0.001 and current_velocity < 0.447:
        #    brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * abs(self.decel_limit)
        #    throttle = 0

        #elif acceleration > 0:
        #    throttle = acceleration
        #    brake = 0
        #else:
        #    brake = (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius * abs(acceleration)
        #    throttle = 0

        if current_velocity < 0.1 and linear_velocity == 0.:
            throttle = 0.
            brake = 700
        elif throttle < 0.1 and cte < 0.:
            throttle = 0.
            decel = max(self.decel_limit, cte)
            brake = self.wheel_radius * self.vehicle_mass * abs(decel)
           
        #rospy.loginfo('my test control-brake:%s throttle:%s', brake, throttle)
        return throttle, brake, steer
