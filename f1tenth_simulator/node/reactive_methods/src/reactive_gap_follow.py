#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

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
        ranges = ranges[np.arange(270,809)]

        print(len(ranges))
        
        proc_ranges = ranges
        return proc_ranges


    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        free_space_ranges = np.array(free_space_ranges)
        
        #for i in range(len(free_space_ranges)):
            
        
        max_index = np.where(free_space_ranges==free_space_ranges.max())
        
        
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
        proc_ranges = self.preprocess_lidar(ranges)

        #Find closest point to LiDAR
        # for i in range(len(proc_ranges)):
        #     if         
        free_space_ranges=[]
        
        min_index_tuple = np.where(proc_ranges==proc_ranges.min())
        
        min_index_int = int(min_index_tuple[0])
        
        min_range = ranges[min_index_int]               ####################################### error
        
        
        theta = math.atan(1.5/min_range)        # 0.3 = 0.6/2 -> 0.6 = bubble width
        angle_index = int(theta / data.angle_increment)
        
        
        #Eliminate all points inside 'bubble' (set them to zero) 
        proc_ranges = proc_ranges.tolist()
        for i in range(len(proc_ranges)):
            if min_index_int - angle_index < i < min_index_int + angle_index : 
                free_space_ranges.append(0.0)
            else :
                free_space_ranges.append(proc_ranges[i])
                
       
        #Find max length gap 
        max_gap = self.find_max_gap(free_space_ranges)


        #Find the best point in the gap 
        
        free_space_ranges = np.array(free_space_ranges)
        max_index_tuple = np.where(free_space_ranges==free_space_ranges.max())
        max_index_int = int(max_index_tuple[0])
        
        print("max distance :" + str(max_index_int))
        print("min distance :" + str(min_index_int))
        print("------------------------------------")
        
        angle_diff_index = 270 - max_index_int
        
        str_angle = angle_diff_index * data.angle_increment
        
        velocity = 2.0
        

        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -str_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
