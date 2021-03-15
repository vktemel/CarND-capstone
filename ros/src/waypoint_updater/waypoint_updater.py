#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from copy import deepcopy

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

LOOKAHEAD_WPS = 70 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        self.next_wp_idx_pub = rospy.Publisher('/next_waypoint_idx', Int32, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.traffic_stop_wp = None

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

        if(ahead > 0):
            return (idx+1) % len(self.base_waypoints)
        else:
            return idx

    def publish_wps(self, idx):
        lane = Lane()
        base_waypoints = self.base_waypoints[idx:idx+LOOKAHEAD_WPS]
        if((self.traffic_stop_wp != None) & (self.traffic_stop_wp > idx)):
            temp_list = []
            tl_idx = (self.traffic_stop_wp - idx) - 5
            tl_wp = base_waypoints[tl_idx]
            
            for wp in base_waypoints[0:tl_idx]:
                temp_wp = Waypoint()
                temp_wp.pose = wp.pose
                dist = self.distance(temp_wp.pose.pose.position, tl_wp.pose.pose.position)
                vel = math.sqrt(2 * MAX_DECEL * dist) 
                if vel < 1.0:
                    vel = 0.0
                temp_wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                
                temp_list.append(temp_wp)
            
            lane.waypoints = temp_list
            
        else:
            lane.waypoints = base_waypoints

        self.next_wp_idx_pub.publish(idx)
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_stop_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    # def distance(self, waypoints, wp1, wp2):
    #     dist = 0
    #     dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    #     for i in range(wp1, wp2+1):
    #         dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
    #         wp1 = i
    #     return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
