#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosa import ROSA, RobotSystemPrompts
from langchain_ollama import ChatOllama
from langchain.agents import tool
#from langchain_core.tools import tool
from puffin_msgs_interfaces.msg import Tutwist
from std_msgs.msg import String
import warnings # Add this to suppress the Pydantic warning

# Suppress the specific Pydantic UserWarning
warnings.filterwarnings("ignore", category=UserWarning, module="pydantic")

@tool
def robot_linear_movement(speed: float, duration: float) -> str:
    """
    This tool is used to control the robot's linear movement. Linear movement refers to moving the robot forwards or backwards.
    Publishes a Tutwist message to /cmd_ollama that moves the robot forward or backward at a specified speed for a specified duration. Requires the speed and duration parameters to be provided.
    Args
    :param speed: The speed that the robot is travelling forwards or backwards.
    :param duration: The duration that the robot travels forwards or backwards at the specified speed.
    """

    # Your code here ...
    Tutwist_msg = Tutwist()
    Tutwist_msg.linear_x = float(speed)
    Tutwist_msg.linear_x_duration = float(duration)
    ollama_node.ollama_publisher.publish(Tutwist_msg)
    ollama_node.get_logger().info(f"Published command: Linear Velocity {Tutwist_msg.linear_x}, Duration {Tutwist_msg.linear_x_duration}")
    return f"ROSA: Attempting to MOVE robot with linear speed of {speed} for {duration} seconds."

@tool
def robot_turning_movement(speed: int, duration: float) -> str:
    """
    This tool is used to control the robot's turning movement.
    Publishes a Tutwist message to /cmd_ollama that turns the robot left or right at a specified angular speed for a specified duration. Requires the angular speed and duration parameters to be provided.
    Args:
    :param speed: The speed that the robot is turning left or right.
    :param duration: The duration that the robot turns left or right at the specified angular speed.
    """
    # Your code here ...
    Tutwist_msg = Tutwist()
    Tutwist_msg.angular_z = float(speed)
    Tutwist_msg.angular_z_duration = float(duration)
    ollama_node.ollama_publisher.publish(Tutwist_msg)
    ollama_node.get_logger().info(f"Published command: Angular Velocity {Tutwist_msg.angular_z}, Duration {Tutwist_msg.angular_z_duration}")
    return f"ROSA: Attempting to TURN robot with angular speed of {speed} for {duration} seconds."

print("Registered tools:", [t.name for t in [robot_linear_movement, robot_turning_movement]])

class OllamaPublisherNode(Node):
    def __init__(self):
        super().__init__('ollama_publisher_node')
        
        self.ollama_publisher = self.create_publisher(Tutwist, '/cmd_ollama', 10)
        self.ollama_subscriber = self.create_subscription(String, '/whisper_transcription', self.callback, 10)
        
        self.get_logger().info("Ollama Publisher Node Initialized")
        
        # Initialize LLM and prompts
        self.ollama_llm = ChatOllama(
            model="llama3.2",
            temperature=0,
            num_ctx=8192,  # Reduced from 8192
            verbose=True,
        )

        self.prompts = RobotSystemPrompts(
            embodiment_and_persona="You are a three-wheeled robot that moves around.",
            critical_instructions=(
                "IMPORTANT: Execute only ONE action per user request unless explicitly told to do multiple actions. "
                "For compound commands like 'go forward and turn right', execute them as separate sequential actions ONLY if the user explicitly requests both. "
                "Do not add extra movements or corrections."
            ),
            about_your_capabilities=(
                "You control movement using two Python functions: `robot_linear_movement` (forwards/backwards) and `robot_turning_movement` (turning). "
                "These functions handle all ROS communication. There is no need to access ROS nodes or topics directly. "
                "EXECUTE EACH FUNCTION ONLY ONCE PER USER COMMAND. "
                "If duration is not specified, use a default of 2 seconds. "
                "If speed is not specified, use 1 for linear and 2 for angular movement. "
                "Turning left uses a positive angular speed. "
                "Turning right uses a negative angular speed. "
                "Moving forward uses a positive linear speed, moving backward uses a negative linear speed. "
                "DO NOT repeat, undo, or correct your actions automatically."
            ),
            about_your_environment=(
                "You are in a ROS environment. Only use the provided movement functions; do not access ROS nodes or topics directly. "
                "Execute each function only once per user request, even if you assume default values."
            ),
            about_your_operators=(
                "Your operator gives movement instructions, which should include a magnitude and duration. "
            ),
        )
        
        # Create an instance of the ROSA agent with the specified prompts and tools
        self.agent = ROSA(ros_version=2, llm=self.ollama_llm, tools=[robot_linear_movement, robot_turning_movement], prompts=self.prompts, verbose=True)
        self.get_logger().info("ROSA Agent Initialized")

    def callback(self, data):
        self.get_logger().info(f"Received transcription: {data.data}")
        agent_response = self.agent.invoke(data.data)
        # self.get_logger().info(f"Agent Final Response (from debug invocation): {self.agent_response}")

# Global variable to access the node from tool functions
ollama_node = None


def main(args=None):
    global ollama_node
    
    rclpy.init(args=args)
    
    try:
        ollama_node = OllamaPublisherNode()
    
        
        #debugging example commands:
        
        #agent_response = ollama_node.agent.invoke("Please turn the robot left 90 degrees, then turn the robot left 90 degrees.")
        #agent_response = ollama_node.agent.invoke("go forward and turn left")
        #ollama_node.get_logger().info(f"Agent Final Response (from debug invocation): {ollama_node.agent_response}")
        #agent_response = ollama_node.agent.invoke("Give me a list of nodes.")
        #agent_response = ollama_node.agent.invoke("robot_linear_movement(speed=1.0, duration=2.0)")
        #ollama_node.get_logger().info(f"Agent Final Response: {agent_response}")
        #print("DEBUG: Agent chat history:", ollama_node.agent.chat_history)
        
        rclpy.spin(ollama_node)
    except KeyboardInterrupt:
        pass
    finally:
        if ollama_node:
            ollama_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
