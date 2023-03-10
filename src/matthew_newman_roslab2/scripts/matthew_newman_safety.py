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

speed = 0

def callback_scan(data):
    try:    
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_inc = data.angle_increment
        angle_n = angle_min
        ii = 0
        TTC_min = float('inf')
        while(angle_n <= angle_max):
            #speed_test = 1
            range_n = data.ranges[ii]
            speed_n = speed * math.cos(angle_n)
            if (speed_n > 0):
                TTC_n = range_n/speed_n
                if(TTC_n < TTC_min) :
                    TTC_min = TTC_n
            #rospy.loginfo(rospy.get_caller_id() + ' speed: %f, angle: %f, speed_n: %f, TTC_n: %f', speed_test, angle_n, speed_n, TTC_min)
            ii += 1
            angle_n += angle_inc
            #rospy.sleep(0.02)
        if(TTC_min < 0.32):
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0.0
            brake_pub.publish(ack_msg)
            brake_bool_pub.publish(True)
            rospy.loginfo(rospy.get_caller_id() + ' TTC_min: %f', TTC_min)
        else:
            brake_bool_pub.publish(False)
        #if(TTC_min != float('inf')):

        #if(numpy.isnan(range_min) or numpy.isnan(range_max) or numpy.isinf(range_min) or numpy.isinf(range_max)):
        #    rospy.loginfo(rospy.get_caller_id() + ' bad entry')                   
        #else:
            #closest_pub.publish(range_min)
            #farthest_pub.publish(range_max)
            #rospy.loginfo(rospy.get_caller_id() + ' min is %f, max is %f' + str(type(range_min)) + ' ' + str(type(range_max)), range_min, range_max)
    except:
        rospy.loginfo(rospy.get_caller_id() + ' error0')  

def callback_odom(data):
    try:       
        global speed    
        speed = data.twist.twist.linear.x
    except:
        rospy.loginfo(rospy.get_caller_id() + ' error1')  

#closest_pub = rospy.Publisher('closest_point', Float64)
#farthest_pub = rospy.Publisher('farthest_point', Float64)  
brake_pub = rospy.Publisher('brake',AckermannDriveStamped, queue_size=10)
brake_bool_pub = rospy.Publisher('brake_bool',Bool, queue_size=10)  


def safety():
    #speed = 0
    rospy.init_node('matthew_newman_safety')
    rospy.loginfo(rospy.get_caller_id() + ' E Brake Monitoring...')
    rospy.Subscriber('scan', LaserScan, callback_scan)
    rospy.Subscriber('odom', Odometry, callback_odom)
    
    rospy.spin()



if __name__ == '__main__':
    try:
        safety()
    except rospy.ROSInternalException:
        pass