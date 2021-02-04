
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

from pid import PID

import rospy


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.throttle = 0

        self.pid_speed = PID(0.1, 0, 0.1)
        self.pid_steer = PID(3.0, 0.0, 0.0)

        self.timestamp = rospy.get_time()

    def control(self, target_linear_vel, target_angular_vel, current_linear_vel, current_angular_vel):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        current_timestamp = rospy.get_time()
        sample_time = current_timestamp - self.timestamp
        self.timestamp = current_timestamp
        speed_diff = target_linear_vel.x - current_linear_vel.x
        throttle = self.pid_speed.step(speed_diff, sample_time)

        rospy.loginfo('throttle command: %s', throttle)

        angular_diff = target_angular_vel.z - current_angular_vel.z
        steer = self.pid_steer.step(angular_diff, sample_time)

        rospy.loginfo('steer command: %s', steer)

        brake = 0.0
        return throttle, brake, steer
