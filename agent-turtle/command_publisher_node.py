#! /usr/bin/env python

# will receive magnitude of linear x and angular z, and duration of each 

import rospy
from std_msgs.msg import String
from agentturtle.msg import tutwist
from geometry_msgs.msg import Twist
import queue

class CommandPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('command_publisher', anonymous=True)

        self.sub = rospy.Subscriber('/cmd_ollama', tutwist, self.callback)

        # Create a publisher for the '/cmd_vel' topic
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Set the rate at which to publish messages
        self.rate = rospy.Rate(10)  # 10 Hz (need to update duration to reduce 0.1s every publish)
        # self.is_publishing = False
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.end_time_linear_x = 0.0
        self.end_time_angular_z = 0.0
        self.publishing_rate = 10  # 10 Hz
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publishing_rate), self.publish_command)
        self.command_queue = queue.Queue()
        self.executing = False

    def callback(self, message):
        rospy.loginfo(f"Ollama Command Received: Linear Velocity of {message.linear_x} for {message.linear_x_duration:.2f} seconds,"
                      f"Angular Velocity of {message.angular_z} for {message.angular_z_duration:.2f} seconds")
        self.command_queue.put(message)
        self.try_execute_next_command()

    def try_execute_next_command(self):
        if not self.executing and not self.command_queue.empty():
            message = self.command_queue.get()
            self.executing = True   

            if message.linear_x_duration > 0:
                self.end_time_linear_x = rospy.get_time() + message.linear_x_duration
                self.current_linear_x = (message.linear_x)/2
            else:
                self.end_time_linear_x = 0.0
                self.current_linear_x = 0.0

            if message.angular_z_duration > 0:
                self.end_time_angular_z = rospy.get_time() + message.angular_z_duration
                self.current_angular_z = (message.angular_z)/2
            else:
                self.end_time_angular_z = 0.0
                self.current_angular_z = 0.0


    def publish_command(self, event):
        if rospy.get_time() >= self.end_time_linear_x:
            self.current_linear_x = 0.0
            self.end_time_linear_x = 0.0  # Reset end time to avoid repeated zero velocity
        if rospy.get_time() >= self.end_time_angular_z:
            self.current_angular_z = 0.0
            self.end_time_angular_z = 0.0
        if self.current_linear_x == 0.0 and self.current_angular_z == 0.0 and self.executing:
            self.executing = False
            self.try_execute_next_command()

        # Create a Twist message
        twist_msg = Twist()

        # Set the linear and angular velocities
        twist_msg.linear.x = self.current_linear_x
        twist_msg.angular.z = self.current_angular_z

        # Publish the Twist message to the '/cmd_vel' topic
        self.pub.publish(twist_msg)

        # Log the published command
        if (self.current_linear_x != 0.0 or self.current_angular_z != 0.0):
            rospy.loginfo(f"Command Publisher : Linear Velocity {self.current_linear_x:.2f} m/s, Angular Velocity {self.current_angular_z:.2f} rad/s")



if __name__ == '__main__':
    try:
        commander = CommandPublisher()
        rospy.spin()  # Keep the node running and listening for messages
    except rospy.ROSInterruptException:
        rospy.loginfo("Command publisher node terminated.")
