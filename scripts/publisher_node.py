#! /usr/bin/env

import rospy
from std_msgs.msg import String
from agentturtle.msg import position

def talk_to_me():
    pub = rospy.Publisher('talking_topic',position , queue_size=10)
    rospy.init_node('publisher_node',anonymous=True)
    rate = rospy.Rate(1)
    rospy.loginfo("publisher node started")
    while not rospy.is_shutdown():
        msg = position()
        msg.message = "my position is: "
        msg.x = 2.0
        msg.y = 1.5
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass
