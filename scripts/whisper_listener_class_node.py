#! /usr/bin/env python

import rospy
import pyaudio
import numpy as np
import whisper
from std_msgs.msg import String
import threading

class WhisperListener:
    def __init__(self):
        rospy.init_node("whisper_listener", anonymous=True)

        # ROS Publisher
        self.transcription_pub = rospy.Publisher('transcription_topic', String, queue_size=10)

        # Whisper Model - Load once at initialization
        rospy.loginfo("Loading Whisper model (tiny)...")
        self.model = whisper.load_model("tiny")
        rospy.loginfo("Whisper model loaded.")

        # Audio Parameters
        self.CHUNK_DURATION = 5
        self.SAMPLE_RATE = 16000
        self.CHUNK_SIZE = int(self.SAMPLE_RATE * self.CHUNK_DURATION)
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1

        # PyAudio objects
        self.p = pyaudio.PyAudio()
        self.stream = None # Will be initialized when opening audio stream

        # State management for transcription
        self.is_transcribing_commands = False
        self.command_timer = None
        self.command_count = 0
        rospy.on_shutdown(self._on_shutdown)

    def _open_audio_stream(self):
        """Opens the PyAudio input stream."""
        if self.stream and self.stream.is_active(): # Close existing stream if any
            self._close_audio_stream()

        try:
            self.stream = self.p.open(format=self.FORMAT,
                                      channels=self.CHANNELS,
                                      rate=self.SAMPLE_RATE,
                                      input=True,
                                      frames_per_buffer=self.CHUNK_SIZE,
                                      stream_callback=None # No callback, we read manually
                                      )
            rospy.loginfo("Audio stream opened successfully.")
            return True
        except Exception as e:
            rospy.logerr(f"Error opening audio stream: {e}")
            self.stream = None
            return False

    def _close_audio_stream(self):
        """Closes the PyAudio input stream."""
        if self.stream:
            try:
                if self.stream.is_active(): # Check if active before stopping
                    self.stream.stop_stream()
                self.stream.close()
                rospy.loginfo("Audio stream stopped and closed.")
            except Exception as e:
                rospy.logwarn(f"Error closing audio stream: {e}")
            finally:
                self.stream = None # Ensure stream is set to None after closing

    def _on_shutdown(self):
        """Callback executed when the ROS node is shutting down."""
        rospy.loginfo("Shutting down WhisperListener node...")
        if self.command_timer:
            self.command_timer.shutdown() # Stop any active timers
            self.command_timer = None
        rospy.sleep(0.1)
        self._close_audio_stream() # Close the audio stream
        if self.p:
            try:
                self.p.terminate()
                rospy.loginfo("PyAudio terminated.")
            except Exception as e:
                rospy.logwarn(f"Error terminating PyAudio during shutdown: {e}")

    def _process_audio_chunk(self):
        """Reads audio data and converts it for Whisper."""
        if not self.stream:
            return None

        try:
            audio_data = self.stream.read(self.CHUNK_SIZE, exception_on_overflow=False)
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
            audio_np = whisper.pad_or_trim(audio_np, whisper.audio.N_SAMPLES)
            return audio_np
        except Exception as e:
            rospy.logerr(f"Error reading audio data: {e}")
            self._close_audio_stream() 
            return None

    def _transcribe_audio(self, audio_np):
        """Transcribes audio using the Whisper model."""
        rospy.loginfo("Transcribing audio, not recording")
        try:
            result = self.model.transcribe(audio_np, language='en', fp16=False) # fp16=False for CPU
            return result['text'].strip()
        except Exception as e:
            rospy.logerr(f"Error during transcription: {e}")
            return ""

    def _handle_command_timer(self, event):
        """Callback for the command mode timer."""
        self.command_count += 1
        rospy.loginfo(f"Command timer incremented: {self.command_count}")
        if self.command_count >= 30:
            rospy.loginfo("Stopping command transcription after 30 seconds of inactivity.")
            self.command_timer.shutdown()
            self.command_timer= None
            self.is_transcribing_commands = False

    def start_command_transcription(self):
        """Starts the command transcription mode."""
        rospy.loginfo("Entering command transcription mode. Say a command!")
        self.is_transcribing_commands = True
        self.command_count = 0

        if not self.stream:
            rospy.logerr("Failed to open audio stream for command mode. Cannot proceed.")
            self.is_transcribing_commands = False # Ensure loop doesn't start
            self.wait_for_hello_loop() # Immediately go back to waiting for wake word if this fails
            return

        if self.command_timer: # Ensure no old timer is running
            self.command_timer.shutdown()
        self.command_timer = rospy.Timer(rospy.Duration(1), self._handle_command_timer)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown() and self.is_transcribing_commands:
            rospy.loginfo("Waiting for command...")
            audio_np = self._process_audio_chunk()
            command = self._transcribe_audio(audio_np)

            if command: # Only process if something was transcribed
                rospy.loginfo(f"Transcribed command: '{command}'")
                processed_command = ""
                # Check for keywords and publish
                if "left" in command.lower():
                    processed_command = "left"
                elif "right" in command.lower():
                    processed_command = "right"
                elif "forward" in command.lower():
                    processed_command = "forward"
                elif "backward" in command.lower():
                    processed_command = "backward"
                elif "stop" in command.lower():
                    processed_command = "stop"

                if processed_command:
                    rospy.loginfo(f"Received valid command: {processed_command}")
                    self.transcription_pub.publish(processed_command)
                    self.command_count = 0 # Reset timer on valid command
                else:
                    rospy.loginfo(f"No recognized command in: '{command}'")
            else:
                rospy.loginfo("No audio data processed or transcription failed.")
            rate.sleep()

        # If loop exits (either shutdown or is_transcribing_commands becomes False)
        rospy.loginfo("Exited command transcription mode.")
        self._close_audio_stream()
        


    def wait_for_hello_loop(self):
        """Main loop to listen for 'hello turtle'."""
        rospy.loginfo("Listening for 'hello turtle'...")
        self.is_transcribing_commands = False # Ensure this is false

        # Ensure the stream is opened correctly *before* starting the main loop

        self._open_audio_stream() # Ensure stream is open

        if not self.stream:
            rospy.logerr("Failed to open audio stream for wake word. Node may not function.")
            # If we can't open stream, we can't listen. Keep trying or let node crash.
            # For now, let's keep trying in the loop, but it will error on process_audio_chunk.
            # A better solution here might be to spin ROS or wait, then retry.
            pass # Continue to the loop, where _process_audio_chunk will log errors

        rate = rospy.Rate(1) # Process chunks at 1 Hz

        while not rospy.is_shutdown() and not self.is_transcribing_commands:
            rospy.loginfo("Waiting for 'hello'...")
            audio_np = self._process_audio_chunk()
            transcription = self._transcribe_audio(audio_np)

            rospy.loginfo(f"Heard: '{transcription}'")

            if "hello" in transcription.lower(): # Using .lower() for robustness
                rospy.loginfo("Received 'hello turtle'!")
                self.start_command_transcription()
            rate.sleep()

        rospy.loginfo("Exited wake word listening loop.")
        self._close_audio_stream() # Ensure stream is closed upon loop exit


    def run(self):
        """Main entry point to start the node's operation."""
        # Initial state: waiting for "hello turtle"
        self.wait_for_hello_loop()

if __name__ == '__main__':
    try:
        listener = WhisperListener()
        listener.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Whisper listener node interrupted.")