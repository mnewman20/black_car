#!/usr/bin/env python  
#Author: Matthew Newman
#Revised: 4/1/23
#File: range_finder.py
#Course: CSCE 860
#Assignment: Lab 2.3
#Description: Create range finder node for f1tenth car. Use lidar to find nearest point to lidar, transform
#the position to the base_link frame, and calculate the minimum time to collision (TTC). publish TTC_min.

import rospy
import math
import tf2_ros
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import numpy

class RangeFinder(object):
    def __init__(self) -> None:
        self.car_radius = 0.3 
        self.resolution = 5
        self.speed = 0.1
        rospy.wait_for_service('static_map')#wait for simulator to start
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(0.5)   #wait for buffer to start getting data
        try:
            self.trans = self.tfBuffer.lookup_transform('base_link', 'laser', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo(rospy.get_caller_id() + ' error2') 

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom) 
        rospy.sleep(0.5)   #wait for odometry to start getting data

        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.callback_scan)
        self.TTC_pub = rospy.Publisher('TTC_min', Float64, queue_size=10)

        rospy.loginfo(rospy.get_caller_id() + ' starting range finder...')  

    def callback_scan(self, data):
        try:    
            #only look at points in front/behind of the car. other points will not be collided with
            angle_min = data.angle_min
            angle_max = data.angle_max
            angle_inc = data.angle_increment
            angle_n = angle_min
            ii = 0
            TTC_out = float('inf')
            #only check TTC if moving, else output TTC=inf
            #save current speed to speed_n to avoid odom overwriting value mid loop
            speed_n = self.speed
            if(abs(speed_n) >= 0.1):
                while(angle_n <= angle_max):
                    #only care if lidar point is in front/behind car
                    pose_y = data.ranges[ii]*math.sin(angle_n) + self.trans.transform.translation.y
                    if(abs(pose_y) <= self.car_radius):
                        #if point is in front of car, calculate x distance
                        pose_x = data.ranges[ii]*math.cos(angle_n) + self.trans.transform.translation.x
                        #only check TTC if the point is in the same direction as travel
                        if(numpy.sign(pose_x) == numpy.sign(speed_n)):
                            #use x distance to determine TTC for point n
                            TTC_n = pose_x/speed_n
                            #if TTC_n is less than the currently smallest TTC, update TTC_out
                            if(TTC_n < TTC_out) :
                                TTC_out = float(TTC_n)
                    ii += self.resolution
                    angle_n += angle_inc*self.resolution
            self.TTC_pub.publish(TTC_out)
        except:
            rospy.loginfo(rospy.get_caller_id() + ' error0')  

    def callback_odom(self, data):
        try:       
            self.speed = data.twist.twist.linear.x
        except:
            rospy.loginfo(rospy.get_caller_id() + ' error1')  

if __name__ == '__main__':
    try:
        rospy.init_node('range_finder_node')
        range_finder = RangeFinder()
        rospy.spin()

    except rospy.ROSInternalException:
        pass

   