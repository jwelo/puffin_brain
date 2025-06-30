#!/usr/bin/env python3

"""
Complete launch file for all puffin_brain nodes
This launch file starts all the main nodes in the puffin_brain package
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        # Log information about the launch
        LogInfo(
            msg='Starting puffin_brain complete system...'
        ),

        # Whisper Listener Node - handles speech recognition
        Node(
            package='puffin_brain',
            executable='whisper_listener_node',
            name='whisper_listener',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # Ollama Publisher Node - handles AI language model integration
        Node(
            package='puffin_brain',
            executable='ollama_publisher_node',
            name='ollama_publisher',
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        # Command Publisher Node - handles command processing and publishing
        Node(
            package='puffin_brain',
            executable='command_publisher_node',
            name='command_publisher',
            output='screen',
            respawn=False,
            respawn_delay=2.0
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
