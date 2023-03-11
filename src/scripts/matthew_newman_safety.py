#!/usr/bin/env python

import rospy
import numpy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
#from geometry_msgs.msg import PoseWithCovariance as PoseC
#from geometry_msgs.msg import TwistWithCovariance as TwistC
from ackermann_msgs.msg import AckermannDriveStamped 

speed = 0.5
TTC_threshold = 2.0

#def callback_threshold(data):
#    try:
#        global TTC_threshold 
#        TTC_threshold = data
#        rospy.loginfo(rospy.get_caller_id() + ' TTC_threshold: %f', TTC_threshold)
#    except:
#        rospy.loginfo(rospy.get_caller_id() + ' error1')  


def callback_scan(data):
    try:    
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_inc = data.angle_increment
        angle_n = angle_min
        ii = 0
        TTC_min = float('inf')
        while(angle_n <= angle_max):
            range_n = data.ranges[ii]
            speed_n = speed * math.cos(angle_n)
            if (speed_n > 0):
                TTC_n = range_n/speed_n
                if(TTC_n < TTC_min) :
                    TTC_min = TTC_n
            ii += 1
            angle_n += angle_inc
        bag_pub.publish(TTC_min)
        bag_pub_2.publish(TTC_threshold)
        if(TTC_min < TTC_threshold):
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0.0
            brake_pub.publish(ack_msg)
            #brake_bool_pub.publish(True)
            rospy.loginfo(rospy.get_caller_id() + ' TTC_min: %f', TTC_min)
        #else:
            #brake_bool_pub.publish(False)
        #if(TTC_min != float('inf')):

        #if(numpy.isnan(range_min) or numpy.isnan(range_max) or numpy.isinf(range_min) or numpy.isinf(range_max)):
        #    rospy.loginfo(rospy.get_caller_id() + ' bad entry')                   
        #else:
            #closest_pub.publish(range_min)
            #farthest_pub.publish(range_max)
            #rospy.loginfo(rospy.get_caller_id() + ' min is %f, max is %f' + str(type(range_min)) + ' ' + str(type(range_max)), range_min, range_max)
    except:
        rospy.loginfo(rospy.get_caller_id() + ' error0')  

#def callback_odom(data):
#    try:       
#        global speed    
#        speed = data.twist.twist.linear.x
#    except:
#        rospy.loginfo(rospy.get_caller_id() + ' error1')  

brake_pub = rospy.Publisher('vesc/high_level/ackermann_cmd_mux/input/nav_0',AckermannDriveStamped, queue_size=10)
#brake_bool_pub = rospy.Publisher('brake_bool',Bool, queue_size=10)  
bag_pub = rospy.Publisher('TTC_min',Float64, queue_size=10)
bag_pub_2 = rospy.Publisher('TTC_threshold', Float64, queue_size=10)

def safety():
    rospy.init_node('safety')
    rospy.loginfo(rospy.get_caller_id() + ' E Brake Monitoring...')
    rospy.Subscriber('scan', LaserScan, callback_scan)
    #rospy.Subscriber('TTC_threshold', Float64, callback_threshold)
    #rospy.Subscriber('odom', Odometry, callback_odom)
    
    rospy.spin()



if __name__ == '__main__':
    try:
        safety()
    except rospy.ROSInternalException:
        pass
