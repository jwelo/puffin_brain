#! /usr/bin/env python

import rospy
from std_msgs.msg import String

rospy.init_node('test_agent-turtle')

publisher = rospy.Publisher('/say_hello', String, queue_size=1)
rate = rospy.Rate(3) # 3hz

while not rospy.is_shutdown():
    publisher.publish('Hey!')
    rate.sleep()
