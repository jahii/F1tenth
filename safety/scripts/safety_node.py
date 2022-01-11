#!/usr/bin/env python
from re import S
import rospy                                            # rospy is needed to writing ROS node 
from std_msgs.msg import *                              # to publish message which type is std_msgs.msg/String
from sensor_msgs.msg import LaserScan                   # to publish message which type is sensor_msgs.msg/LaserScam
from nav_msgs.msg import Odometry                       # to publish message which type is nav_msgs.msg/Odometry
from ackermann_msgs.msg import AckermannDriveStamped    # to publish message which type is ackermann_msg/AckermannderiveStamped
from math import *                                      # math is needed to use calculation function and (pi,e)                             

class Safety(object): #The class that handles emergency braking.

    def __init__(self):
        
        # node subscribes to the odom topic which is of type nav_msgs/Odometry. 
        rospy.Subscriber("odom", Odometry, self.odom_callback) 
        # node subscribes to the scan topic which is of type sensor_msgs/LaserScan. 
        rospy.Subscriber("scan", LaserScan, self.scan_callback) #receive 'scan' topic
        
        # node is publishing to the brake_bool topic using the message type Bool
        self.pub_brake_bool= rospy.Publisher("brake_bool", Bool, queue_size=10)
        # node is publishing to the /brake topic using the message type AckermannDrivestamped
        self.pub_brake = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=10)
        
        self.threshold = 0.3
        self.breakspeed = 0
        


    def odom_callback(self, odom_msg):
        self.odom = odom_msg # receive odom_msg        

    def scan_callback(self, scan_msg):
        
        self.scan=scan_msg   # receive scan_msg
        
        angle_min=scan_msg.angle_min                # Lidar minumum angle: -2.355rad
        angle_increment = scan_msg.angle_increment  # Laser angle increment: 4.71/1080[rad], scan beams:1080
        ranges = scan_msg.ranges                    # Distance between vehicle and obstacle
        velocity = self.odom.twist.twist.linear.x   # vehicle velocity
       
        brake_bool_msg=Bool()                       # message which type is Bool
        brake_bool_msg.data=False                   # update parameter 'data' in brake_bool_msg as False
        
        brake_msg=AckermannDriveStamped()           # message which type is AckermannDriveStamped
        brake_msg.drive.speed=self.breakspeed       # update parameter 'speed' in brake_msg as 0
        
        
        for i in range(len(ranges)):                # for every laser point
            theta = angle_min + angle_increment * i # calculate theta based on angle_min(-2.355rad)
            
            if velocity != 0:                       
                velocity = abs(velocity)                            # to be robust from noise(negative)
                if -pi/10 < theta < pi/10:                          # set the FOV(Field of View)
                    TTC = ranges[i] / (velocity*cos(theta))         # Cacluate TTC
                    if TTC < self.threshold:                        # if TTC is smaller than threshold, then vehicle must be stopped
                        brake_bool_msg.data=True                    # update parameter 'data' in brake_bool_msg as False
                        self.pub_brake_bool.publish(brake_bool_msg) # publish brake_bool_msg
                        self.pub_brake.publish(brake_msg)           # publish brake_msg
                                     
        brake_bool_msg.data=False   
        self.pub_brake_bool.publish(brake_bool_msg)
        
       
def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()
if __name__ == '__main__':
    main()
