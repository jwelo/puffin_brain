#! /usr/bin/env python

import rospy
import pyaudio
import numpy as np
import whisper
from std_msgs.msg import String

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

    def _open_audio_stream(self):
        """Opens the PyAudio input stream."""
        if self.stream: # Close existing stream if any
            self._close_audio_stream()

        try:
            self.stream = self.p.open(format=self.FORMAT,
                                      channels=self.CHANNELS,
                                      rate=self.SAMPLE_RATE,
                                      input=True,
                                      frames_per_buffer=self.CHUNK_SIZE)
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
                self.stream.stop_stream()
                self.stream.close()
                rospy.loginfo("Audio stream stopped and closed.")
            except Exception as e:
                rospy.logwarn(f"Error closing audio stream: {e}")
            finally:
                self.stream = None # Ensure stream is set to None after closing

    def _terminate_pyaudio(self):
        """Terminates the PyAudio instance."""
        if self.p:
            try:
                self.p.terminate()
                rospy.loginfo("PyAudio terminated.")
            except Exception as e:
                rospy.logwarn(f"Error terminating PyAudio: {e}")
            finally:
                self.p = None # Ensure PyAudio object is set to None

    def _process_audio_chunk(self):
        """Reads audio data and converts it for Whisper."""
        if not self.stream:
            rospy.logerr("Audio stream is not open. Cannot process audio.")
            return None

        try:
            audio_data = self.stream.read(self.CHUNK_SIZE, exception_on_overflow=False)
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
            audio_np = whisper.pad_or_trim(audio_np, whisper.audio.N_SAMPLES)
            return audio_np
        except Exception as e:
            rospy.logerr(f"Error reading audio data: {e}")
            return None

    def _transcribe_audio(self, audio_np):
        """Transcribes audio using the Whisper model."""
        if audio_np is None:
            return ""
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
        if self.command_count >= 5:
            rospy.loginfo("Stopping command transcription after 5 seconds of inactivity.")
            self.command_timer.shutdown()
            self.is_transcribing_commands = False
            self.wait_for_hello_loop() # Go back to waiting for "hello turtle"

    def start_command_transcription(self):
        """Starts the command transcription mode."""
        self.is_transcribing_commands = True
        self.command_count = 0
        if self.command_timer: # Ensure no old timer is running
            self.command_timer.shutdown()
        self.command_timer = rospy.Timer(rospy.Duration(1), self._handle_command_timer)

        rospy.loginfo("Entering command transcription mode. Say a command!")
        self._open_audio_stream() # Ensure stream is open

        rate = rospy.Rate(1) # 1 Hz

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

            rate.sleep()

        # If loop exits (either shutdown or is_transcribing_commands becomes False)
        self._close_audio_stream()
        self._terminate_pyaudio()
        rospy.loginfo("Exited command transcription mode.")


    def wait_for_hello_loop(self):
        """Main loop to listen for 'hello turtle'."""
        self.is_transcribing_commands = False
        self._open_audio_stream() # Ensure stream is open for this mode

        rospy.loginfo("Listening for 'hello turtle'...")
        rate = rospy.Rate(1) # 1 Hz

        while not rospy.is_shutdown() and not self.is_transcribing_commands:
            rospy.loginfo("Waiting for 'hello turtle'...")
            audio_np = self._process_audio_chunk()
            transcription = self._transcribe_audio(audio_np)

            rospy.loginfo(f"Heard: '{transcription}'")

            if "hello turtle" in transcription.lower(): # Using .lower() for robustness
                rospy.loginfo("Received 'hello turtle'!")
                self.start_command_transcription()
                break # Exit this loop to start command transcription
            rate.sleep()

        if rospy.is_shutdown():
            self._close_audio_stream()
            self._terminate_pyaudio()

    def run(self):
        """Main entry point to start the node's operation."""
        # Initial state: waiting for "hello turtle"
        self.wait_for_hello_loop()
        rospy.spin() # Keep the node alive if wait_for_hello_loop exits due to shutdown

if __name__ == '__main__':
    try:
        listener = WhisperListener()
        listener.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Whisper listener node interrupted.")
    finally:
        # Ensure PyAudio is terminated even if an error occurs
        if 'listener' in locals() and listener.p:
            listener._terminate_pyaudio()