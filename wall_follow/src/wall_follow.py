#!/usr/bin/env python
from __future__ import print_function
import sys
from math import *
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan                            # to publish message which type is sensor_msgs/(Image,LaserScan)
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive    # to publish message which type is ackermann_msg/(AckermannderiveStamped,AckermannDrive)

kp = 14    
kd = 0.09   
ki = 0      
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270               # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9    # desired distance from right wall
DESIRED_DISTANCE_LEFT = 0.8     # desired distance from left wall
VELOCITY = 1.00                 # vehicle velocity [m/s]
CAR_LENGTH = 0.50 
THETA = pi/10

class WallFollow:
    
    def __init__(self):
        
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        # node subscribes to the scan topic which is of type sensor_msgs/LaserScan.
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        # node is publishing to the /brake topic using the message type AckermannDrivestamped
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        

    def lidar_callback(self, data):
        
        laser_data = data
        angle = pi/5
        
        error = self.followLeft(laser_data, DESIRED_DISTANCE_LEFT)
        #0.0 #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)   # set the steering angle for the VESC of vehicle using PID Control


    def followLeft(self, data, leftDist):
        
        a, b = self.getRange(data, THETA)               # get the ditstance between vehicle and two random points of right wall 
        
        alpha = atan((a*cos(THETA)-b) / a*sin(THETA))   # get the angle between two distance vector
        
        D_cur = b*cos(alpha)                            # current distance to wall from ego vehicle
        D_fut = D_cur + CAR_LENGTH*sin(alpha)           # future distance to wall from ego vehicle
        
        caculated_error = D_fut - leftDist              # set error used in PID control
        return caculated_error

    def getRange(self, data, angle):                    # get the ditstance between vehicle and two random points of right wall 
        
        b = data.ranges[269]    
        
        i = int(angle / (2*pi/1080))        
        
        a = data.ranges[269+i]
        
        return a,b
     
    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        
        angle = kp * error + kd * (error - prev_error)
        
        drive_msg = AckermannDriveStamped()         # message which type is AckermannDriveStamped
        #drive_msg.header.stamp = rospy.Time.now()   # message which type is AckermannDriveStamped
        
        drive_msg.header.frame_id = "laser"         # update parameter frame_id in drive_msg as "laser"
        drive_msg.drive.steering_angle = -angle     # update parameter steering_angle in drive_msg as -angle
        drive_msg.drive.speed = velocity            # update parameter speed in drive_msg as velocity
        
        self.drive_pub.publish(drive_msg)           # publish drive_msg
        
        print(error)
        prev_error = error                          # set previous error


def main(args):
    rospy.init_node("WallFollow_node")
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
