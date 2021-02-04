
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID
from yaw_controller import YawController

import rospy


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.throttle = 0

        self.pid_speed = PID(0.2, 0, 0.1)

        self.yawCtrl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.timestamp = rospy.get_time()

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

        rospy.loginfo('throttle command: %s', throttle)

        steer = self.yawCtrl.get_steering(target_linear_vel.x, target_angular_vel.z, current_linear_vel.x)

        rospy.loginfo('steer command: %s', steer)

        brake = 0.0
        return throttle, brake, steer
