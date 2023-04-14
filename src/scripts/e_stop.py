#!/usr/bin/env python  
#Author: Matthew Newman
#Revised: 4/1/23
#File: e_stop.py
#Course: CSCE 860
#Assignment: Lab 2.3
#Description: Create emergency stop node for f1tenth car. Subscribe to TTC_min and if it is below
#a given threshold, publish 'brake' speed as zero and 'brake_bool as True. 

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped 

class EStop(object):
    def __init__(self) -> None:
        rospy.loginfo(rospy.get_caller_id() + ' E Brake Monitoring...')
        self.TTC_min = rospy.Subscriber('TTC_min', Float64, self.callback_TTC)
        self.brake_pub = rospy.Publisher('brake',AckermannDriveStamped, queue_size=10)
        self.brake_bool_pub = rospy.Publisher('brake_bool',Bool, queue_size=10)

    def callback_TTC(self, data):
        if(data.data <= 0.35):
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0.0
            self.brake_pub.publish(ack_msg)
            self.brake_bool_pub.publish(True)
            rospy.loginfo(rospy.get_caller_id() + ' TTC_min: %f', data.data)
        else:
            self.brake_bool_pub.publish(False)

if __name__ == '__main__':
    try:
        rospy.init_node('e_stop_node')
        e_stop = EStop()
        rospy.spin()

    except rospy.ROSInternalException:
        pass