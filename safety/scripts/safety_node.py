#!/usr/bin/env python
from re import S
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from math import *

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        rospy.Subscriber("odom", Odometry, self.odom_callback) #receive 'odom' topic
        rospy.Subscriber("scan", LaserScan, self.scan_callback) #receive 'scan' topic
        
        self.pub_brake_bool= rospy.Publisher("brake_bool", Bool, queue_size=10)
        self.pub_brake = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=10)
        self.threshold = 0.5
        
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a Bool message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        # TODO: create ROS subscribers and publishers.

    def odom_callback(self, odom_msg):
        self.odom = odom_msg
        # TODO: update current speed
        

    def scan_callback(self, scan_msg):
        angle_min=scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        
        ranges = scan_msg.ranges
        velocity = self.odom.twist.twist.linear.x
       
        brake_bool_msg=Bool()
        brake_bool_msg.data=False
        
        brake_msg=AckermannDriveStamped() #create brake_msg format
        brake_msg.drive.speed=self.speed
        
        # TODO: calculate TTC
        for i in range(len(ranges)):
            theta = angle_min + angle_increment * i
            if velocity != 0:
                velocity = abs(velocity)
                if -pi/10 < theta < pi/10:
                    TTC = ranges[i] / (velocity*cos(theta))
                    print(TTC)
                    #print("theta : ", theta)
                    if TTC < self.threshold:
                        brake_bool_msg.data=True
                        self.pub_brake_bool.publish(brake_bool_msg)
                        brake_bool_msg.data=False
                        
                        # self.pub_brake_bool.publish(brake_bool_msg)
                        # self.pub_brake.publish(brake_msg)
                        print(brake_bool_msg.data)
                        
                        
            
        self.pub_brake_bool.publish(brake_bool_msg)
        self.pub_brake.publish(brake_msg)
        # print(angle_increment)
        
        
        # rospy.loginfo("scan data : %f"%velocity)

        
        

        # TODO: publish brake message and publish controller bool
        


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
