#!/usr/bin/env python  
#Author: Matthew Newman
#Revised: 4/8/23
#File: wall_following.py
#Course: CSCE 860
#Assignment: Lab 3.1
#Description: Create PID controller to find distance to wall and drive error to zero. 

import rospy
import math
import tf2_ros
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped 
import numpy

class WallFollower(object):
    def __init__(self) -> None:
        #try:
            self.simulation = True # run code fifferent in simulation
            self.follow_left = True #if true, follow left wall, otherwise follow right wall
            #scan from right of car (-90) over a 70deg arc (-20) to get angle of the wall relative to car
            if(self.follow_left):
                self.scan_range_0 = 90*math.pi/180
                self.scan_range_1 = 20*math.pi/180
                self.scan_range_2 = 0*math.pi/180
                self.scan_range_3 = -30*math.pi/180
            else:
                self.scan_range_0 = -90*math.pi/180
                self.scan_range_1 = -20*math.pi/180
                self.scan_range_2 = 0*math.pi/180
                self.scan_range_3 = 30*math.pi/180
            if(self.simulation):
                self.kp = 1 #pid P term
                self.ki = 0 #pid I term
                self.kd = 0 #pid D term
                self.distance_target = 1#target distance from wall
                self.look_ahead = 0.9 #fraction of time interval between scans to look ahead
                self.min_turn_radius = 1.25
                self.continuous_wall_range = 5.0
            else:
                self.kp = 1 #pid P term
                self.ki = 0 #pid I term
                self.kd = 0 #pid D term
                self.distance_target = 1#target distance from wall
                self.look_ahead = 0.9 #fraction of time interval between scans to look ahead
                self.min_turn_radius = 0.75

            self.speed = 0.
            self.distance_sum = 0

            #rospy.wait_for_service('static_map')#wait for simulator to start

            #self.tfBuffer = tf2_ros.Buffer()
            #self.listener = tf2_ros.TransformListener(self.tfBuffer)
            #rospy.sleep(0.25)   #wait for buffer to start getting data
            #self.trans = self.tfBuffer.lookup_transform('base_link', 'laser', rospy.Time(0))
            
            self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom) 

            rospy.wait_for_message('scan', LaserScan)
            self.t_last_scan = rospy.get_time()
            scan_info = rospy.wait_for_message('scan', LaserScan)
            self.t_current_scan = rospy.get_time()
            
            #find the index for the values in scan associated with 0 and +70 degrees to obtain two wall points
            #to determine angle from wall
            self.angle_zero_index = max(int((self.scan_range_0 - scan_info.angle_min)/scan_info.angle_increment),0)
            self.angle_one_index = max(int((self.scan_range_1-scan_info.angle_min)/scan_info.angle_increment),0)
            self.angle_two_index = max(int((self.scan_range_2 - scan_info.angle_min)/scan_info.angle_increment),0)
            self.angle_three_index = max(int((self.scan_range_3 - scan_info.angle_min)/scan_info.angle_increment),0)

            if(self.follow_left):
                self.c_theta = math.sin(self.scan_range_1)#cos(70deg) wrt axis pointing to the wall
                self.s_theta = math.cos(self.scan_range_1)#sin(70deg) wrt axis pointing to the wall
            else:
                self.c_theta = -math.sin(self.scan_range_1)#cos(70deg) wrt axis pointing to the wall
                self.s_theta = math.cos(self.scan_range_1)#sin(70deg) wrt axis pointing to the wall
            
            self.find_distance_t(scan_info)
           
            self.scan_sub = rospy.Subscriber('scan', LaserScan, self.callback_scan)
            if(self.simulation):
                self.drive_pub = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)
            else:
                self.drive_pub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0',AckermannDriveStamped, queue_size=10)
            self.distance_target_pub = rospy.Publisher('distance_target', Float64, queue_size=10)
            self.distance_current_pub = rospy.Publisher('distance_current', Float64, queue_size=10)
            rospy.sleep(.25)   #wait for odometry to start getting data

            rospy.loginfo(rospy.get_caller_id() + ' starting wall following...')  
        #except:
        #    rospy.loginfo(rospy.get_caller_id() + ' error0')  

   
    def callback_scan(self, data):
        #try:   
            self.t_last_scan = self.t_current_scan
            self.t_current_scan = rospy.get_time()
            self.distance_t1 = self.distance_t
            override = self.find_distance_t(data)
            self.distance_target_pub.publish(self.distance_target)
            self.distance_current_pub.publish(self.distance_t)
            self.controller()
            self.drive(override)

        #except:
        #    rospy.loginfo(rospy.get_caller_id() + ' error1')  

    def callback_odom(self, data):
        try:       
            self.speed = data.twist.twist.linear.x
        except:
            rospy.loginfo(rospy.get_caller_id() + ' error2')  

    def find_distance_t(self, data):
        r_0 = data.ranges[self.angle_zero_index]
        r_theta = data.ranges[self.angle_one_index]
        alpha = math.atan((r_theta*self.c_theta-r_0)/(r_theta*self.s_theta))
        distance_t = r_0*math.cos(alpha)
        distance_t_p1 = distance_t + self.speed * (self.t_current_scan - self.t_last_scan) * math.sin(alpha)
        self.distance_t = distance_t + self.look_ahead * distance_t_p1
        self.distance_sum += self.distance_t * (self.t_current_scan - self.t_last_scan) #integrate area under distance:time curve for PID control
        #check if wall is in front of opposing side of car within the turn radius. If so, send an override signal turn sharply the opposing direction
        opposing_angle_index = min(self.angle_two_index, self.angle_three_index)
        while (opposing_angle_index <= max(self.angle_two_index, self.angle_three_index)):
            if(data.ranges[opposing_angle_index] < self.min_turn_radius):
                #if opposing point is found, check that it connects to wall you are following
                connection_angle_index = min(opposing_angle_index,self.angle_zero_index)
                while(connection_angle_index <= max(opposing_angle_index,self.angle_zero_index)):
                    if(data.ranges[connection_angle_index] > self.continuous_wall_range):
                        return False
                    connection_angle_index += 1
                #if you scan from the point back to the side with all points in range, ovverride controller
                return True
            opposing_angle_index += 1
        return False


    def controller(self):
        output_p = self.kp * (self.distance_target - self.distance_t) # P = kp * (target - current)
        output_i = self.ki * self.distance_sum
        output_d = self.kd * (self.distance_t1 - self.distance_t)/(self.t_current_scan-self.t_last_scan)# D = kd * (d_t(-1) - d_t(0) / (t(0) - t(-1))
        self.steering_angle = output_p + output_i - output_d

        return self.steering_angle

    def drive(self, override):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = "laser"
        if(override):
            self.speed = 0.5
            ack_msg.drive.speed = self.speed
            if(self.follow_left):
                ack_msg.drive.steering_angle = -1.57
            else:
                ack_msg.drive.steering_angle = 1.57
        else:
            if(abs(self.steering_angle) <= (10/180*math.pi)):#if drive angle < 10deg
                self.speed = 1.5
            elif(abs(self.steering_angle) <= (20/180*math.pi)):#if drive angle < 20deg
                self.speed = 1.0
            else:
                self.speed = 0.5
            ack_msg.drive.speed = self.speed
            if(self.follow_left):
                ack_msg.drive.steering_angle = -self.steering_angle
            else:
                ack_msg.drive.steering_angle = self.steering_angle
        self.drive_pub.publish(ack_msg)
   
if __name__ == '__main__':
    try:
        rospy.init_node('wall_following_node')
        range_finder = WallFollower()
        rospy.spin()

    except rospy.ROSInternalException:
        pass

   