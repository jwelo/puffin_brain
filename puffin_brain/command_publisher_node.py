#!/usr/bin/env python3

# will receive magnitude of linear x and angular z, and duration of each 

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from puffin_msgs_interfaces.msg import Tutwist
from geometry_msgs.msg import Twist
import queue
import time

class CommandPublisher(Node):
    def __init__(self):
        # Initialize the ROS node
        super().__init__('command_publisher')

        self.sub = self.create_subscription(Tutwist, '/cmd_ollama', self.callback, 10)

        # Create a publisher for the '/cmd_vel' topic
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set the rate at which to publish messages
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.end_time_linear_x = 0.0
        self.end_time_angular_z = 0.0
        self.publishing_rate = 10  # 10 Hz
        self.timer = self.create_timer(1.0 / self.publishing_rate, self.publish_command)
        self.command_queue = queue.Queue()
        self.executing = False

    def callback(self, message):
        self.get_logger().info(f"Ollama Command Received: Linear Velocity of {message.linear_x} for {message.linear_x_duration:.2f} seconds,"
                      f"Angular Velocity of {message.angular_z} for {message.angular_z_duration:.2f} seconds")
        self.command_queue.put(message)

    def try_execute_next_command(self):
        if not self.executing and not self.command_queue.empty():
            message = self.command_queue.get()
            self.executing = True   

            if message.linear_x_duration > 0:
                self.end_time_linear_x = time.time() + message.linear_x_duration
                self.current_linear_x = message.linear_x
            else:
                self.end_time_linear_x = 0.0
                self.current_linear_x = 0.0

            if message.angular_z_duration > 0:
                self.end_time_angular_z = time.time() + message.angular_z_duration
                self.current_angular_z = message.angular_z
            else:
                self.end_time_angular_z = 0.0
                self.current_angular_z = 0.0

    def publish_command(self):
        current_time = time.time()
        if not self.executing:
            self.try_execute_next_command()
        else:
            # Check if linear movement time has expired
            if self.end_time_linear_x > 0.0 and current_time >= self.end_time_linear_x:
                self.current_linear_x = 0.0
                self.end_time_linear_x = 0.0
            
            # Check if angular movement time has expired
            if self.end_time_angular_z > 0.0 and current_time >= self.end_time_angular_z:
                self.current_angular_z = 0.0
                self.end_time_angular_z = 0.0
                
            # Check if we're done executing the current command
            if self.current_linear_x == 0.0 and self.current_angular_z == 0.0:
                self.executing = False

        # Create a Twist message
        twist_msg = Twist()

        # Set the linear and angular velocities
        twist_msg.linear.x = self.current_linear_x
        twist_msg.angular.z = self.current_angular_z

        # Publish the Twist message to the '/cmd_vel' topic
        self.pub.publish(twist_msg)

        # Log the published command
        if (self.current_linear_x != 0.0 or self.current_angular_z != 0.0):
            self.get_logger().info(f"Command Publisher : Linear Velocity {self.current_linear_x:.2f} m/s, Angular Velocity {self.current_angular_z:.2f} rad/s")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        commander = CommandPublisher()
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
