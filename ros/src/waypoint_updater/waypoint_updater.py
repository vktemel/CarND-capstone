#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None

        # rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                idx = self.find_next_waypoint(self.pose, self.base_waypoints)
                self.publish_wps(idx)
            rate.sleep()

    def find_next_waypoint(self, pose, base_waypoints):
        # Next waypoint is closest ahead of the vehicle. 
        # Here, closest waypoint will be found, then checked if ahead of vehicle
        # If it's ahead of vehicle, then the closest waypoint will be returned
        # If it's not ahead of the vehicle, then next waypoint will be returned
        idx = 0
        min_distance = 1e5
        x_veh = pose.pose.position.x
        y_veh = pose.pose.position.y
        z_veh = pose.pose.position.z
        for i in range(len(base_waypoints)):
            waypoint = base_waypoints[i]
            x_wp = waypoint.pose.pose.position.x
            y_wp = waypoint.pose.pose.position.y
            z_wp = waypoint.pose.pose.position.z
            distance = math.sqrt((x_veh-x_wp)**2 + (y_veh-y_wp)**2 + (z_veh-z_wp)**2)
            if distance < min_distance:
                min_distance = distance
                idx = i

        # dot product of vectors indicate if it's ahead or not
        next_wp = np.array([base_waypoints[idx].pose.pose.position.x, base_waypoints[idx].pose.pose.position.y])
        prev_wp = np.array([base_waypoints[idx-1].pose.pose.position.x, base_waypoints[idx-1].pose.pose.position.y])
        pos_vect = np.array([x_veh, y_veh])

        ahead = np.dot(next_wp-prev_wp, pos_vect-next_wp)

        if(ahead):
            return idx
        else:
            return idx+1

    def publish_wps(self, idx):
        lane = Lane()
        lane.waypoints = self.base_waypoints[idx:idx+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
