#!/usr/bin/env python
from __future__ import print_function
import sys
from math import *
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 14    #TODO
kd = 0.09   #TODO
ki = 0      #TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8
VELOCITY = 1.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
THETA = pi/10

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)#TODO: Publish to drive
        
    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        
        b = data.ranges[269]
        
        i = int(angle / (2*pi/1080))
        
        a = data.ranges[269+i]
        
        #TODO: implement
        return a,b

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        
        #TODO: Use kp, ki & kd to implement a PID controller for 
        angle = kp * error + kd * (error - prev_error)
        
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        
        print(error)
        prev_error = error

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        a, b = self.getRange(data, THETA)
        
        alpha = atan((a*cos(THETA)-b) / a*sin(THETA))
        
        D_cur = b*cos(alpha)    # D = distance to wallfrom ego vehicle
        D_fut = D_cur + CAR_LENGTH*sin(alpha)
        
        
        #TODO:implement
        caculated_error = D_fut - leftDist
        return caculated_error

    def lidar_callback(self, data):
        """ 
        """
        laser_data = data
        angle = pi/5
        
        
        error = self.followLeft(laser_data, DESIRED_DISTANCE_LEFT)
        #0.0 #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
