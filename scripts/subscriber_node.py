#! /usr/bin/env/ python

import rospy
from std_msgs.msg import String
from agentturtle.msg import position

def callback(message):
    rospy.loginfo("%s X: %f Y: %f", message.message, message.x, message.y)

def listener():
    rospy.init_node("subscriber_node", anonymous=True)
    rospy.Subscriber('talking_topic',position, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

