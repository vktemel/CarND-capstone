
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID
from yaw_controller import YawController

import rospy


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle, decel_limit, wheel_radius, vehicle_mass):
        self.throttle = 0

        self.pid_speed = PID(0.2, 0.001, 0.1)

        self.yawCtrl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.timestamp = rospy.get_time()

        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius
        self.vehicle_mass = vehicle_mass
        
    def reset(self):
        self.pid_speed.reset()

    def control(self, target_linear_vel, target_angular_vel, current_linear_vel, current_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        current_timestamp = rospy.get_time()
        dt = current_timestamp - self.timestamp
        self.timestamp = current_timestamp
        speed_diff = target_linear_vel.x - current_linear_vel.x 
        throttle = self.pid_speed.step(speed_diff, dt)
        if throttle > 1.0:
            throttle = 1.0
        elif throttle < 0:
            throttle = 0

        steer = self.yawCtrl.get_steering(target_linear_vel.x, target_angular_vel.z, current_linear_vel.x)
        
        # rospy.logwarn("target vel x: %s", target_linear_vel.x)

        vel_delta = current_linear_vel.x - target_linear_vel.x

        if((current_linear_vel.x < 0.01) & (target_linear_vel.x == 0.0)):
            throttle = 0
            brake = 700
        elif(vel_delta > 0.1):
            throttle = 0
            decel = max(vel_delta, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
        else:
            brake = 0.0

        return throttle, brake, steer
