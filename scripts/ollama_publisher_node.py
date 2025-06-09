#! /home/evas/turtle_ws/venv_rosa/bin/python

import rospy
from rosa import ROSA, RobotSystemPrompts
from langchain_ollama import ChatOllama
from langchain.agents import tool
#from langchain_core.tools import tool
from agentturtle.msg import tutwist
import warnings # Add this to suppress the Pydantic warning

# Suppress the specific Pydantic UserWarning
warnings.filterwarnings("ignore", category=UserWarning, module="pydantic")

@tool
def robot_linear_movement(speed: float, duration: float) -> str:
    """
    This tool is used to control the robot's linear movement. Linear movement refers to moving the robot forwards or backwards.
    Publishes a tutwist message to /cmd_ollama that moves the robot forward or backward at a specified speed for a specified duration. Requires the speed and duration parameters to be provided.

    :param speed: The speed that the robot is travelling forwards or backwards.
    :param duration: The duration that the robot travels forwards or backwards at the specified speed.
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

    :param speed: The speed that the robot is turning left or right.
    :param duration: The duration that the robot turns left or right at the specified angular speed.
    """

    # Your code here ...
    tutwist_msg = tutwist()
    tutwist_msg.angular_z = speed
    tutwist_msg.angular_z_duration = duration
    ollama_publisher.publish(tutwist_msg)
    rospy.loginfo(f"Published command: Angular Velocity {tutwist_msg.angular_z}, Duration {tutwist_msg.angular_z_duration}")
    return f"ROSA: Attempting to TURN robot with angular speed of {speed} for {duration} seconds."

print("Registered tools:", [t.name for t in [robot_linear_movement, robot_turning_movement]])

ollama_publisher = None  # Initialized the publisher variable in __main__

ollama_llm = ChatOllama(
    model="llama3.1:8b",
    temperature=0,
    # num_ctx=2048,
    verbose=True,
)

prompts = RobotSystemPrompts(
    embodiment_and_persona="You are a cool, three-wheeled robot that moves around.",
    # critical_instructions="You must confirm all actions with the operator before proceeding. Failure to do so might result in damage to the robot or its environment.",
    about_your_capabilities="You have two Python functions to control your movement: `robot_linear_movement` for moving forward/backward, and `robot_turning_movement` for turning. You MUST call these specific functions to move. Do not use any other tools like `rosservice_call` for movement tasks.",
    about_your_environment="You are in a ROS environment, but for movement, you must rely exclusively on the provided Python functions (`robot_linear_movement`, `robot_turning_movement`). These functions handle the underlying ROS communication for you.",
    about_your_operators="Your operator is a human who will provide you with instructions on how to move. The instructions should include a movement magnitude and duration. If either parameter is missing, you should ask the operator for clarification."
)

if __name__ == "__main__":
    rospy.init_node('ollama_publisher_node', anonymous=True)
    ollama_publisher = rospy.Publisher('/cmd_ollama', tutwist, queue_size=10)
    rospy.loginfo("Ollama Publisher Node Initialized")

    # Create an instance of the ROSA agent with the specified prompts and tools
    agent = ROSA(ros_version=1, llm=ollama_llm, tools=[robot_linear_movement, robot_turning_movement], prompts=prompts, verbose=True)
    print("Agent tools:", [t.name for t in agent._ROSA__tools.get_tools()])
    rospy.loginfo("Invoking agent with command...")
    try:
        #agent_response = agent.invoke("Please move the robot forward at speed 1.0 for 2.0 seconds using the provided movement tools. Are you able to access the two custom tools that I have written for you?")
        #agent_response = agent.invoke("Give me a list of nodes.")
        agent_response = agent.invoke("robot_linear_movement(speed=1.0, duration=2.0)")
        rospy.loginfo(f"Agent Final Response: {agent_response}")
        print("DEBUG: Agent chat history:", agent.chat_history)
    except Exception as e:
        rospy.logerr(f"An error occurred during agent invocation: {e}")
    
    rospy.loginfo("ready!")
    rospy.spin()
    