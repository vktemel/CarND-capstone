#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 200

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.next_wp_idx = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub_wpidx = rospy.Subscriber('/next_waypoint_idx', Int32, self.waypoint_idx_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def waypoint_idx_cb(self, msg):
        self.next_wp_idx = msg

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        state = self.state
        
        if (self.waypoints is not None) & (self.next_wp_idx is not None):
            light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x_position, y_position):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        tmp_dist = 1e8
        wp_idx = None
        for i in range(len(self.waypoints.waypoints)):
            wp_x = self.waypoints.waypoints[i].pose.pose.position.x
            wp_y = self.waypoints.waypoints[i].pose.pose.position.y
            dist = math.sqrt((x_position-wp_x)**2 + (y_position-wp_y)**2)
            if dist < tmp_dist:
                tmp_dist = dist
                wp_idx = i
        return wp_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        return light.state
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # #Get classification
        # return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            if(self.next_wp_idx is not None):
                car_wp = self.next_wp_idx.data
            else:
                car_wp = -1
            next_wp = self.waypoints.waypoints[car_wp]
            final_wp = self.waypoints.waypoints[car_wp+LOOKAHEAD_WPS]

            #TODO find the closest visible traffic light (if one exists)
            min_dist = 1e8
            light_idx = None
            for i in range(len(self.lights)):
                line_pos = stop_line_positions[i]
                line_x = line_pos[0]
                line_y = line_pos[1]

                next_wp_x = next_wp.pose.pose.position.x
                next_wp_y = next_wp.pose.pose.position.y

                dist = math.sqrt((line_x-next_wp_x)**2 + (line_y-next_wp_y)**2)
                if(dist < min_dist):
                    min_dist = dist
                    light_idx = i
            
            # line_pos, next_wp_pos, final_wp_pos
            next_wp_vect = np.array([next_wp.pose.pose.position.x, next_wp.pose.pose.position.y])
            final_wp_vect = np.array([final_wp.pose.pose.position.x, final_wp.pose.pose.position.y])
            line_pos_vect = np.array([stop_line_positions[light_idx][0], stop_line_positions[light_idx][1]])

            found_light_ahead = np.dot(line_pos_vect-next_wp_vect, final_wp_vect-line_pos_vect)
            if(found_light_ahead > 0):
                light = self.lights[i]

        if light:
            state = self.get_light_state(light)
            light_wp = self.get_closest_waypoint(stop_line_positions[light_idx][0], stop_line_positions[light_idx][1])
            return light_wp, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
