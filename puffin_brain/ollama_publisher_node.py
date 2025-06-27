#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosa import ROSA, RobotSystemPrompts
from langchain_ollama import ChatOllama
from langchain.agents import tool
#from langchain_core.tools import tool
from puffin_msgs_interface.msg import tutwist
from std_msgs.msg import String
import warnings # Add this to suppress the Pydantic warning

# Suppress the specific Pydantic UserWarning
warnings.filterwarnings("ignore", category=UserWarning, module="pydantic")

@tool
def robot_linear_movement(speed: int, duration: float) -> str:
    """
    This tool is used to control the robot's linear movement. Linear movement refers to moving the robot forwards or backwards.
    Publishes a tutwist message to /cmd_ollama that moves the robot forward or backward at a specified speed for a specified duration. Requires the speed and duration parameters to be provided.
    Args:
        speed: The speed that the robot is travelling forwards or backwards.
        duration: The duration that the robot travels forwards or backwards at the specified speed.
    """

    # Your code here ...
    tutwist_msg = tutwist()
    tutwist_msg.linear_x = speed
    tutwist_msg.linear_x_duration = duration
    ollama_node.ollama_publisher.publish(tutwist_msg)
    ollama_node.get_logger().info(f"Published command: Linear Velocity {tutwist_msg.linear_x}, Duration {tutwist_msg.linear_x_duration}")
    return f"ROSA: Attempting to MOVE robot with linear speed of {speed} for {duration} seconds."

@tool
def robot_turning_movement(speed: int, duration: float) -> str:
    """
    This tool is used to control the robot's turning movement.
    Publishes a tutwist message to /cmd_ollama that turns the robot left or right at a specified angular speed for a specified duration. Requires the angular speed and duration parameters to be provided.
    Args:
        speed: The speed that the robot is turning left or right.
        duration: The duration that the robot turns left or right at the specified angular speed.
    """
    # Your code here ...
    tutwist_msg = tutwist()
    tutwist_msg.angular_z = speed
    tutwist_msg.angular_z_duration = duration
    ollama_node.ollama_publisher.publish(tutwist_msg)
    ollama_node.get_logger().info(f"Published command: Angular Velocity {tutwist_msg.angular_z}, Duration {tutwist_msg.angular_z_duration}")
    return f"ROSA: Attempting to TURN robot with angular speed of {speed} for {duration} seconds."

print("Registered tools:", [t.name for t in [robot_linear_movement, robot_turning_movement]])

class OllamaPublisherNode(Node):
    def __init__(self):
        super().__init__('ollama_publisher_node')
        
        self.ollama_publisher = self.create_publisher(tutwist, '/cmd_ollama', 10)
        self.ollama_subscriber = self.create_subscription(String, '/whisper_transcription', self.callback, 10)
        
        self.get_logger().info("Ollama Publisher Node Initialized")
        
        # Initialize LLM and prompts
        self.ollama_llm = ChatOllama(
            model="llama3.2",
            temperature=0,
            num_ctx=8192,
            verbose=True,
        )

        self.prompts = RobotSystemPrompts(
            embodiment_and_persona="You are a three-wheeled robot that moves around.",
            # critical_instructions="You must confirm all actions with the operator before proceeding. Failure to do so might result in damage to the robot or its environment.",
            about_your_capabilities=(
                "You control movement using two Python functions: `robot_linear_movement` (forwards/backwards) and `robot_turning_movement` (turning). "
                "These functions handle all ROS communication. There is no need to access ROS nodes or topics directly. "
                "If duration is not specified, use a default of 2 seconds. "
                "If speed is not specified, use 1 for linear and 1 for angular movement. "
                #"Call each function only ONCE per user request. "
                "Never repeat, correct, or reflect on, summarize, or undo your actions unless the user asks."
                "Do not perform any actions except those directly requested by the user. "
                "Turning left uses a positive angular speed, turning right uses a negative angular speed."
                "If turning is provided in angles, every 90 degrees is equivalent to speed 2 for two seconds."
                "Moving forward uses a positive linear speed, moving backward uses a negative linear speed. "
                #"You have two Python functions to control your movement: `robot_linear_movement` for moving forward/backward, "
                #"and `robot_turning_movement` for turning. "
                #"You do not need to access the nodes and topics in the ROS environment, as these functions handle the underlying ROS communication for you. "
                #"If the user does not specify a duration, always use a default duration of 2 seconds. If the duration argument is missing, insert the default value into the tool before executing it, so that you only use each tool once per user request. "
                #"You must only call each function ONCE per user request, even if you need to assume a default value."
                #"Never repeat or correct your actions unless the user explicitly asks for a correction or repeat. "
                #"Do not reflect on or review your actions after executing them."
            ),
            about_your_environment=(
                "You are in a ROS environment. Only use the provided movement functions; do not access ROS nodes or topics directly. "
                "Execute each function only once per user request, even if you assume default values."
                #"You are in a ROS environment, but for movement, you must rely exclusively on the provided Python functions (`robot_linear_movement`, `robot_turning_movement`). These functions handle the underlying ROS communication for you. There is no need to find the available topics or nodes. You must only execute each function ONCE per user request, even if you have to assume a default value. "
            ),
            about_your_operators=(
                "Your operator gives movement instructions. "
                #"If duration is missing, use 2 seconds. If speed is missing or zero, use 1. "
                #"Turning left uses a positive angular speed, turning right uses a negative angular speed."

                #"Your operator is a human who will provide you with instructions on how to move. "
                #"The instructions should include a movement magnitude and duration."
                #"If duration is not specified, assume a default duration of 2 seconds. "
                #"If speed is zero OR not specified, assume a default speed of 1 for linear movement and 1 speed for angular movement. "
                #"Do not repeat or correct your actions unless the user asks."
            ),
        )
        
        # Create an instance of the ROSA agent with the specified prompts and tools
        self.agent = ROSA(ros_version=2, llm=self.ollama_llm, tools=[robot_linear_movement, robot_turning_movement], prompts=self.prompts, verbose=True)
        self.get_logger().info("ROSA Agent Initialized")

    def callback(self, data):
        self.get_logger().info(f"Received transcription: {data.data}")
        agent_response = self.agent.invoke(data.data)

# Global variable to access the node from tool functions
ollama_node = None


def main(args=None):
    global ollama_node
    
    rclpy.init(args=args)
    
    try:
        ollama_node = OllamaPublisherNode()
        
        """
        debugging example commands:
        try:
            #agent_response = ollama_node.agent.invoke("Please move the robot forward at speed 1.0 for 2.0 seconds using the provided movement tools.")
            # agent_response = ollama_node.agent.invoke("Please move the robot forward at speed 5, then turn the robot left at speed 1.0 for 5.0 seconds using the provided movement tools.")
            #agent_response = ollama_node.agent.invoke("Give me a list of nodes.")
            #agent_response = ollama_node.agent.invoke("robot_linear_movement(speed=1.0, duration=2.0)")
            # ollama_node.get_logger().info(f"Agent Final Response: {agent_response}")
            # print("DEBUG: Agent chat history:", ollama_node.agent.chat_history)
        except Exception as e:
            ollama_node.get_logger().error(f"An error occurred during agent invocation: {e}")
        """
        
        rclpy.spin(ollama_node)
    except KeyboardInterrupt:
        pass
    finally:
        if ollama_node:
            ollama_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
