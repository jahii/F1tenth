#!/usr/bin/env python
from __future__ import print_function
import sys
import math

import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


# kp = 0.08
# kd = 0.02
# prev_error = 0.0


class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback)             #TODO
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=10)            #TODO
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        
        for i in range(len(ranges)):
            if ranges[i]==np.nan:
                ranges[i]=0.0
        
        ranges = np.array(ranges)
        ranges = ranges[np.arange(269,809)]
        
        proc_ranges = ranges
        return proc_ranges


    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        free_space_ranges = np.array(free_space_ranges)
        max_index = np.where(free_space_ranges==free_space_ranges.max())[0][0]
        #error=max_index-180
        print("max_index : "+str(max_index))
        angle = self.angle_increment*(max_index-269)
        # global prev_error
        # global kp
        # global kd
        
        velocity = 1
        
        
        # #TODO: Use kp, ki & kd to implement a PID controller for 
        # angle = kp * error + kd * (error - prev_error)
        
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        
        
        # prev_error = error
        
        
        
        
        return None
    
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        
        
        return None


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        self.angle_increment = data.angle_increment
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        # for i in range(len(proc_ranges)):
        #     if 
        #150 index safety_bubble
        
        free_space_ranges=[]
        min_index = np.where(proc_ranges==proc_ranges.min())[0][0]
        print("min index : " + str(min_index))
        proc_ranges = proc_ranges.tolist()
        for i in range(len(proc_ranges)):
            if min_index - 60 < i < min_index + 60 : 
                free_space_ranges.append(0.0)
            else :
                free_space_ranges.append(proc_ranges[i])
                
        self.find_max_gap(free_space_ranges)
    
        
        

        #Eliminate all points inside 'bubble' (set them to zero) 
        

        #Find max length gap 


        #Find the best point in the gap 
        

        #Publish Drive message
        

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
