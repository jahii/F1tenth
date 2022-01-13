#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import itertools,operator

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

global prev_steering_angle
prev_steering_angle = 0
global t_prev
t_prev = 0.0
global prev_range
prev_range = []
class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.drive_msg = AckermannDriveStamped()
        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback) #None #TODO
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=5) #TODO
    	

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """


        return 0

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """

        return 0
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """

        return 0

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges = data.ranges

        a = ranges
        forward_angle_min = -100.0
        forward_angle_max = 100.0
        ind_start 	= int((len(a)-1)*(forward_angle_min+180.0)/360.0)
        ind_end 	= int((len(a)-1)*(forward_angle_max+180.0)/360.0)
        b = np.array(a)
        b[b>2.0] = np.NaN


        #Find closest point to LiDAR
        min_b_index = np.nanargmin(b)
        
        #Eliminate all points inside 'bubble' (set them to zero)
        bubblerad = 10
        b[min_b_index-bubblerad:min_b_index+bubblerad] = 0.0
        
    
        if len(b[ind_start:min_b_index-bubblerad]) > len(b[min_b_index+bubblerad:ind_end]):
            max_gap_indstart = ind_start
            max_gap_indend = min_b_index - bubblerad
        else:
            max_gap_indstart = min_b_index+bubblerad
            max_gap_indend = ind_end

    	#Find the best point in the gap
        steering_angle_ind = np.nanargmax(b[max_gap_indstart:max_gap_indend]) + max_gap_indstart
        
        steering_angle = ((steering_angle_ind*360.0/(1079))-180.0)*np.pi/180.0 # in rads
        maxspeed = 3.0

        # can change maxspeed if near wall
        speed = maxspeed
 
        #Publish Drive message
        self.drive_msg.header.stamp = rospy.Time.now()
        self.drive_msg.drive.steering_angle = steering_angle
        self.drive_msg.drive.speed = speed
        self.drive_pub.publish(self.drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)