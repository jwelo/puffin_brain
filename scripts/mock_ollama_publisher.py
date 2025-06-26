#! /usr/bin/env python

import rospy
from std_msgs.msg import String
from agentturtle.msg import tutwist

def mock_publisher():
    # Initialize the ROS node
    rospy.init_node('mock_ollama_publisher', anonymous=True)

    # Create a publisher for the '/cmd_ollama' topic
    pub = rospy.Publisher('/cmd_ollama', tutwist, queue_size=10)

    # Set the rate at which to publish messages
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        # Create a tutwist message
        message = tutwist()
        message.linear_x = 0.5  # Example linear velocity
        message.linear_x_duration = 2.0  # Duration in seconds
        message.angular_z = 0.1  # Example angular velocity
        message.angular_z_duration = 5.0  # Duration in seconds

        # Log the message being published
        rospy.loginfo(f"Publishing: Linear Velocity {message.linear_x}, Duration {message.linear_x_duration}, "
                      f"Angular Velocity {message.angular_z}, Duration {message.angular_z_duration}")

        # Publish the message
        pub.publish(message)

        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        mock_publisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass