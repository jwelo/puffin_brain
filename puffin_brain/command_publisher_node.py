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

        self.sub_ollama = self.create_subscription(Tutwist, '/cmd_ollama', self.ollama_callback, 10)
        self.sub_hand = self.create_subscription(Twist, '/cmd_hand', self.hand_callback, 10)

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
        #self.executing = False
        self.twist_hand_msg = Twist()
        self.ollama_executing = False
        self.hand_executing = False
        self.hand_command_timeout = 0.0


    def ollama_callback(self, message):
        self.get_logger().info(f"Ollama Command Received: Linear Velocity of {message.linear_x} for {message.linear_x_duration:.2f} seconds,"
                      f"Angular Velocity of {message.angular_z} for {message.angular_z_duration:.2f} seconds")
        self.command_queue.put(message)

    def hand_callback(self, message):
        if message.linear.x == 0.0 and message.angular.z == 0.0:
            self.get_logger().warn("Hand Gesture STOP Command Received.")
            self.twist_hand_msg.linear.x = 0.0
            self.twist_hand_msg.angular.z = 0.0
            self.pub.publish(self.twist_hand_msg)
            self.ollama_executing = False
            self.hand_executing = False
            self.hand_command_timeout = 0.0
            # Reset the command queue
            while not self.command_queue.empty():
                self.command_queue.get()
            return
        elif not self.ollama_executing:
            # Set a timeout for hand gestures (e.g., 0.1 seconds)
            self.hand_executing = True
            self.hand_command_timeout = time.time() + 0.2
            
            if message.angular.z > 0.0:
                self.get_logger().info(f"Hand Gesture LEFT Command Received")
                self.twist_hand_msg.linear.x = 0.0
                self.twist_hand_msg.angular.z = 0.5
                self.pub.publish(self.twist_hand_msg)
            elif message.angular.z < 0.0:
                self.get_logger().info(f"Hand Gesture RIGHT Command Received")
                self.twist_hand_msg.linear.x = 0.0
                self.twist_hand_msg.angular.z = -0.5
                self.pub.publish(self.twist_hand_msg)
            elif message.linear.x > 0.0:
                self.get_logger().info(f"Hand Gesture FORWARD Command Received")
                self.twist_hand_msg.linear.x = 0.11
                self.twist_hand_msg.angular.z = 0.0
                self.pub.publish(self.twist_hand_msg)
            else:
                self.get_logger().info(f"Hand Gesture BACKWARD Command Received")
                self.twist_hand_msg.linear.x = -0.11
                self.twist_hand_msg.angular.z = 0.0
                self.pub.publish(self.twist_hand_msg)

    def try_execute_next_command(self):
        if not self.command_queue.empty():
            message = self.command_queue.get()  
            self.ollama_executing = True
            if message.linear_x_duration > 0:
                self.end_time_linear_x = time.time() + message.linear_x_duration
                self.current_linear_x = message.linear_x/23
            else:
                self.current_linear_x = 0.0

            if message.angular_z_duration > 0:
                self.end_time_angular_z = time.time() + message.angular_z_duration
                self.current_angular_z = message.angular_z/2
            else:
                self.current_angular_z = 0.0

    def publish_command(self):
        current_time = time.time()
        
        # Handle hand gesture timeout
        if self.hand_executing and current_time >= self.hand_command_timeout:
            # Stop the hand gesture by publishing zero velocities
            self.hand_executing = False
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.pub.publish(stop_msg)
            #DEBUG self.get_logger().info("Hand gesture timeout - stopped")
            
        if not self.ollama_executing:
            self.try_execute_next_command()
        else:
            # Check if linear movement time has expired
            if current_time >= self.end_time_linear_x:
                self.current_linear_x = 0.0
            
            # Check if angular movement time has expired
            if current_time >= self.end_time_angular_z:
                self.current_angular_z = 0.0
                
            # Check if we're done executing the current command
            if self.current_linear_x == 0.0 and self.current_angular_z == 0.0:
                self.ollama_executing = False

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
    commander = None
    
    try:
        commander = CommandPublisher()
        rclpy.spin(commander)
    except KeyboardInterrupt:
        if commander:
            commander.get_logger().info("Shutting down CommandPublisher...")
    except Exception as e:
        if commander:
            commander.get_logger().error(f"Error in CommandPublisher: {e}")
    finally:
        # Ensure proper cleanup sequence
        if commander:
            try:
                commander.destroy_node()
            except Exception as e:
                print(f"Error destroying commander node: {e}")
        
        # Only shutdown if we initialized and ROS is still ok
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS shutdown: {e}")

if __name__ == '__main__':
    main()
