#! /home/evas/turtle_ws/venv_rosa/bin/python

import rospy
from rosa import ROSA, RobotSystemPrompts
from langchain_ollama import ChatOllama
from langchain.agents import tool
#from langchain_core.tools import tool
from agentturtle.msg import tutwist
from std_msgs.msg import String
import warnings # Add this to suppress the Pydantic warning

# Suppress the specific Pydantic UserWarning
warnings.filterwarnings("ignore", category=UserWarning, module="pydantic")

@tool
def robot_linear_movement(speed: float, duration: float) -> str:
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
    ollama_publisher.publish(tutwist_msg)
    rospy.loginfo(f"Published command: Linear Velocity {tutwist_msg.linear_x}, Duration {tutwist_msg.linear_x_duration}")
    return f"ROSA: Attempting to MOVE robot with linear speed of {speed} for {duration} seconds."

@tool
def robot_turning_movement(speed: float, duration: float) -> str:
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
    ollama_publisher.publish(tutwist_msg)
    rospy.loginfo(f"Published command: Angular Velocity {tutwist_msg.angular_z}, Duration {tutwist_msg.angular_z_duration}")
    return f"ROSA: Attempting to TURN robot with angular speed of {speed} for {duration} seconds."

print("Registered tools:", [t.name for t in [robot_linear_movement, robot_turning_movement]])

ollama_llm = ChatOllama(
    model="llama3.2",
    temperature=1,
    num_ctx=8192,
    verbose=True,
)

prompts = RobotSystemPrompts(
    embodiment_and_persona="You are a cool, three-wheeled robot that moves around.",
    # critical_instructions="You must confirm all actions with the operator before proceeding. Failure to do so might result in damage to the robot or its environment.",
    about_your_capabilities=(
        "You have two Python functions to control your movement: `robot_linear_movement` for moving forward/backward, "
        "and `robot_turning_movement` for turning. "
        "You do not need to access the nodes and topics in the ROS environment, as these functions handle the underlying ROS communication for you. "
        "If the user does not specify a duration, always use a default duration of 2 seconds. If the duration argument is missing, insert the default value into the tool before executing it, so that you only use each tool once per user request. "
        "You must only call each function ONCE per user request, even if you need to assume a default value."
        "Never repeat or correct your actions unless the user explicitly asks for a correction or repeat. "
        "Do not reflect on or review your actions after executing them."
    ),
    about_your_environment="You are in a ROS environment, but for movement, you must rely exclusively on the provided Python functions (`robot_linear_movement`, `robot_turning_movement`). These functions handle the underlying ROS communication for you. There is no need to find the available topics or nodes. You must only execute each function ONCE per user request, even if you have to assume a default value. ",
    about_your_operators=(
        "Your operator is a human who will provide you with instructions on how to move. "
        "The instructions should include a movement magnitude and duration. "
        "If duration is not specified, assume a default duration of 2 seconds. "
        "Do not repeat or correct your actions unless the user asks."
    ),
)
agent = None  # Initialize the agent variable globally
def callback(data):
    global agent
    rospy.loginfo(f"Received transcription: {data.data}")
    agent_response = agent.invoke(data.data)


if __name__ == "__main__":
    rospy.init_node('ollama_publisher_node', anonymous=True)
    ollama_publisher = rospy.Publisher('/cmd_ollama', tutwist, queue_size=10)
    ollama_subscriber = rospy.Subscriber('/whisper_transcription', String, callback)
    rospy.loginfo("Ollama Publisher Node Initialized")

    # Create an instance of the ROSA agent with the specified prompts and tools
    agent = ROSA(ros_version=1, llm=ollama_llm, tools=[robot_linear_movement, robot_turning_movement], prompts=prompts, verbose=True)
    # print("Agent tools:", [t.name for t in agent._ROSA__tools.get_tools()])
    rospy.loginfo("ROSA Agent Initialized")
    """
    debugging example commands:
    try:
        #agent_response = agent.invoke("Please move the robot forward at speed 1.0 for 2.0 seconds using the provided movement tools.")
        # agent_response = agent.invoke("Please move the robot forward at speed 5, then turn the robot left at speed 1.0 for 5.0 seconds using the provided movement tools.")
        #agent_response = agent.invoke("Give me a list of nodes.")
        #agent_response = agent.invoke("robot_linear_movement(speed=1.0, duration=2.0)")
        # rospy.loginfo(f"Agent Final Response: {agent_response}")
        # print("DEBUG: Agent chat history:", agent.chat_history)
    except Exception as e:
        rospy.logerr(f"An error occurred during agent invocation: {e}")
    """
    rospy.spin()
    