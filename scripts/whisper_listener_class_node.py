#! /usr/bin/env python

import rospy
import pyaudio
import numpy as np
import whisper
from std_msgs.msg import String
import threading
import collections
import time

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
        self.CHUNK_DURATION = 1
        self.SAMPLE_RATE = 16000
        # self.CHUNK_SIZE = int(self.SAMPLE_RATE * self.CHUNK_DURATION) (multithreading no longer use)
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1

        # PyAudio objects
        self.p = pyaudio.PyAudio()
        self.stream = None # Will be initialized when opening audio stream

        # State management for transcription
        self.is_transcribing_commands = False
        self.command_timer = None
        self.command_count = 0

        # threading setup
        self.audio_buffer = collections.deque(maxlen=self.SAMPLE_RATE * 10) # To store raw audio samples from recording thread
        self.buffer_lock = threading.Lock() # Protect access to buffer
        self.audio_thread_stop_event = threading.Event()
        self.audio_thread = threading.Thread(target=self._audio_recorder_thread, args=())
        self.audio_thread.daemon = True # Allow main program to exit even if thread is running
        self.THREAD_READ_CHUNK_SIZE = int(self.SAMPLE_RATE * 0.1) # 100 ms chunks for reading audio

        # proper shut down 
        rospy.on_shutdown(self._on_shutdown)

    def _open_audio_stream_for_thread(self):
        """Opens the PyAudio input stream."""
        if self.stream: # Close existing stream if any
            return

        try:
            self.stream = self.p.open(format=self.FORMAT,
                                      channels=self.CHANNELS,
                                      rate=self.SAMPLE_RATE,
                                      input=True,
                                      frames_per_buffer=self.THREAD_READ_CHUNK_SIZE,
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

        self.audio_thread_stop_event.set()
        if self.audio_thread.is_alive():
            rospy.loginfo("Waiting for audio recording thread to finish...")
            self.audio_thread.join(timeout=5) # Wait up to 5 seconds for thread to finish
            if self.audio_thread.is_alive():
                rospy.logwarn("Audio recording thread did not stop gracefully.")

        self._close_audio_stream() # double check that the audio stream is closed
        rospy.loginfo("Audio stream closed.")

        if self.p:
            try:
                self.p.terminate()
                rospy.loginfo("PyAudio terminated.")
            except Exception as e:
                rospy.logwarn(f"Error terminating PyAudio during shutdown: {e}")

    def _audio_recorder_thread(self):
        """
        Dedicated thread to continuously read audio from PyAudio and populate the buffer.
        """
        rospy.loginfo("Audio recorder thread started.")
        self._open_audio_stream_for_thread() # Open stream
        if not self.stream:
            rospy.logerr("Failed to open audio stream in recorder thread. Exiting thread.")
            return

        while not self.audio_thread_stop_event.is_set() and not rospy.is_shutdown():
            try:
                audio_data = self.stream.read(self.THREAD_READ_CHUNK_SIZE, exception_on_overflow=False)
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

                with self.buffer_lock: # Protect buffer access
                    self.audio_buffer.extend(audio_np)
                    # Keep buffer to a manageable size, e.g., 3 seconds of audio (16000 * 3 samples)
                    # This prevents indefinite memory growth while providing enough history.
                    max_buffer_samples = self.SAMPLE_RATE * 3 
            except Exception as e:
                rospy.logerr(f"Error in audio recorder thread: {e}")
                time.sleep(0.5) # Small sleep to prevent busy loop on error

        rospy.loginfo("Audio recorder thread stopping.")
        # self._close_audio_stream() # Close the stream managed by this thread

    def _get_audio_for_transcription(self, desired_duration_seconds):
        """
        Pulls a chunk of audio from the buffer for transcription.
        This will get the MOST RECENT audio up to desired_duration_seconds.
        """
        num_samples = int(self.SAMPLE_RATE * desired_duration_seconds)
        with self.buffer_lock:
            # Ensure we don't try to get more samples than available
            samples_to_get = min(num_samples, len(self.audio_buffer))
            
            # Get the last 'samples_to_get' from the buffer
            current_audio = np.array(list(self.audio_buffer)[-samples_to_get:])
            
        if current_audio.size == 0:
            rospy.logwarn("Audio buffer is empty or not enough samples for transcription.")
            return None

        padded_audio = whisper.pad_or_trim(current_audio, whisper.audio.N_SAMPLES)
        return padded_audio


    def _transcribe_audio(self, audio_np):
        """Transcribes audio using the Whisper model."""
        rospy.loginfo("Transcribing audio, not recording")
        if audio_np is None:
            rospy.logwarn("No audio data provided for transcription.")
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
        if self.command_count >= 15:
            rospy.loginfo("Stopping command transcription after 15 seconds of inactivity.")
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
            return

        if self.command_timer: # Ensure no old timer is running
            self.command_timer.shutdown()
        self.command_timer = rospy.Timer(rospy.Duration(1), self._handle_command_timer)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown() and self.is_transcribing_commands:
            rospy.loginfo("Waiting for command...")
            audio_np = self._get_audio_for_transcription(self.CHUNK_DURATION)
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

    def wait_for_hello_loop(self):
        """Main loop to listen for 'hello turtle'."""
        rospy.loginfo("Listening for 'hello turtle'...")
        self.is_transcribing_commands = False # Ensure this is false

        # Ensure the stream is opened correctly *before* starting the main loop

        if not self.stream:
            rospy.logerr("Failed to open audio stream for wake word. Node may not function.")
            # If we can't open stream, we can't listen. Keep trying or let node crash.
            # For now, let's keep trying in the loop, but it will error on process_audio_chunk.
            # A better solution here might be to spin ROS or wait, then retry.
            pass # Continue to the loop, where _process_audio_chunk will log errors

        rate = rospy.Rate(1) # Process chunks at 1 Hz

        while not rospy.is_shutdown() and not self.is_transcribing_commands:
            rospy.loginfo("Waiting for 'hello'...")
            
            audio_np = self._get_audio_for_transcription(self.CHUNK_DURATION)
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
        self.audio_thread.start()
        time.sleep(1)
        self.wait_for_hello_loop()

if __name__ == '__main__':
    try:
        listener = WhisperListener()
        listener.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Whisper listener node interrupted.")