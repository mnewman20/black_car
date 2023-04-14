#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import numpy
#from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

class TTCFinder(object):
    def __init__(self) -> None:
        super.__init__('TTC_finder_node')
        self.speed = 0
        self.closest_sub = rospy.Subscriber('closest_point', geometry_msgs.msg.TransformStamped, self.callback_transform)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.callback_odom)
        self.TTC_pub = rospy.Publisher('TTC', Float64, queue_size=10)


    def callback_transform(self, data):
        pose2_x = data.transform.translation.x 
        pose2_y = data.transform.translation.y 
        range2 = math.sqrt(pose2_x**2 + pose2_y**2)
        if(self.speed == 0):
            TTC = 1000
        else:
            TTC = range2/self.speed
        self.TTC_pub.publish(TTC)

    def callback_odom(self, data):
        try:       
            self.speed = data.twist.twist.linear.x
        except:
            rospy.loginfo(rospy.get_caller_id() + ' error1')  



if __name__ == '__main__':
    try:
        rospy.init_node('TTC_finder_node')
        ttc_finder = TTCFinder()
        rospy.spin()    

    except rospy.ROSInternalException:
        pass

   