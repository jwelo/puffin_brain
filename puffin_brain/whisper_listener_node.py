#!/usr/bin/env python3
# bruh need to chnage my output 
import rclpy
from rclpy.node import Node
import pyaudio
import numpy as np
from faster_whisper import WhisperModel
import scipy.signal
#import whisper
from std_msgs.msg import String
import threading
import collections
import time


class WhisperListener(Node):
    def __init__(self):
        super().__init__('whisper_listener')

        # ROS Publisher
        self.transcription_pub = self.create_publisher(String, '/whisper_transcription', 10)

        
        # Faster Whisper
        self.get_logger().info("Loading Whisper model (tiny.en)...")
        self.model = WhisperModel("tiny.en", device="cpu", compute_type="int8")
        self.get_logger().info("Whisper model loaded successfully.")
        """
        # Whisper 
        self.model = whisper.load_model("tiny.en")
        self.get_logger().info("Whisper model loaded.")
        """
        # Audio Parameters
        self.CHUNK_DURATION = 1
        self.SAMPLE_RATE = 48000  # Use 48kHz which is supported by ATR4697-USB
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

        # proper shut down - as ROS 2 doesn't have on_shutdown, we'll handle it in main
        self._shutdown_requested = False

    def _open_audio_stream_for_thread(self):
        """Opens the PyAudio input stream."""
        if self.stream: # Close existing stream if any
            return

        try:
            # First, let's list available audio devices
            self.get_logger().info("Available audio devices:")
            usb_device_index = None
            for i in range(self.p.get_device_count()):
                device_info = self.p.get_device_info_by_index(i)
                self.get_logger().info(f"Device {i}: {device_info['name']}, channels: {device_info['maxInputChannels']}")
                # Look for ATR4697-USB device first (better quality), then any USB device with input capabilities
                if "ATR4697" in device_info['name'] and device_info['maxInputChannels'] > 0:
                    usb_device_index = i
                    self.get_logger().info(f"Found ATR4697-USB audio device at index {i}")
                    break
                elif usb_device_index is None and "USB" in device_info['name'] and device_info['maxInputChannels'] > 0:
                    usb_device_index = i
                    self.get_logger().info(f"Found USB audio device at index {i}")
            
            # Try to open with USB device if found, otherwise use default
            if usb_device_index is not None:
                self.get_logger().info(f"Attempting to use USB device {usb_device_index}")
                self.stream = self.p.open(format=self.FORMAT,
                                          channels=self.CHANNELS,
                                          rate=self.SAMPLE_RATE,
                                          input=True,
                                          input_device_index=usb_device_index,
                                          frames_per_buffer=self.THREAD_READ_CHUNK_SIZE,
                                          stream_callback=None
                                          )
            else:
                self.get_logger().info("No USB device found, using default")
                self.stream = self.p.open(format=self.FORMAT,
                                          channels=self.CHANNELS,
                                          rate=self.SAMPLE_RATE,
                                          input=True,
                                          frames_per_buffer=self.THREAD_READ_CHUNK_SIZE,
                                          stream_callback=None
                                          )
            
            self.get_logger().info("Audio stream opened successfully.")
            return True
        except Exception as e:
            self.get_logger().error(f"Error opening audio stream: {e}")
            self.stream = None
            return False

    def _close_audio_stream(self):
        """Closes the PyAudio input stream."""
        if self.stream:
            try:
                if self.stream.is_active(): # Check if active before stopping
                    self.stream.stop_stream()
                self.stream.close()
                print("Audio stream stopped and closed.")
            except Exception as e:
                print(f"Error closing audio stream: {e}")
            finally:
                self.stream = None # Ensure stream is set to None after closing

    def _on_shutdown(self):
        """Callback executed when the ROS node is shutting down."""
        print("Shutting down WhisperListener node...")
        self._shutdown_requested = True
        
        if self.command_timer:
            self.command_timer.cancel() # Stop any active timers from triggering, resources released on node destroy

        self.audio_thread_stop_event.set()
        if self.audio_thread.is_alive():
            print("Waiting for audio recording thread to finish...")
            self.audio_thread.join(timeout=5) # Wait up to 5 seconds for thread to finish
            if self.audio_thread.is_alive():
                print("Audio recording thread did not stop gracefully.")

        self._close_audio_stream() # double check that the audio stream is closed
        print("Audio stream closed.")

        if self.p:
            try:
                self.p.terminate()
                print("PyAudio terminated.")
            except Exception as e:
                print(f"Error terminating PyAudio during shutdown: {e}")

    def _audio_recorder_thread(self):
        """
        Dedicated thread to continuously read audio from PyAudio and populate the buffer.
        """
        self.get_logger().info("Audio recorder thread started.")
        self._open_audio_stream_for_thread() # Open stream
        if not self.stream:
            self.get_logger().error("Failed to open audio stream in recorder thread. Exiting thread.")
            return

        while not self.audio_thread_stop_event.is_set() and not self._shutdown_requested:
            try:
                audio_data = self.stream.read(self.THREAD_READ_CHUNK_SIZE, exception_on_overflow=False)
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0 
                with self.buffer_lock: # Protect buffer access
                    self.audio_buffer.extend(audio_np)
                    # Keep buffer to a manageable size, e.g., 3 seconds of audio (16000 * 3 samples)
                    # This prevents indefinite memory growth while providing enough history.
                    max_buffer_samples = self.SAMPLE_RATE * 3 
            except Exception as e:
                self.get_logger().error(f"Error in audio recorder thread: {e}")
                time.sleep(0.5) # Small sleep to prevent busy loop on error

        print("Audio recorder thread stopping.")

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
            
        # self.get_logger().info(f"Audio buffer size: {len(self.audio_buffer)}, samples requested: {num_samples}, samples retrieved: {samples_to_get}")
        
        if current_audio.size == 0:
            self.get_logger().warn("Audio buffer is empty or not enough samples for transcription.")
            return None

        # Resample from 48000 Hz to 16000 Hz for Whisper
        if self.SAMPLE_RATE != 16000:
            target_samples = int(len(current_audio) * 16000 / self.SAMPLE_RATE)
            current_audio = scipy.signal.resample(current_audio, target_samples).astype(np.float32)
            #self.get_logger().info(f"Resampled audio from {self.SAMPLE_RATE}Hz to 16000Hz, new length: {len(current_audio)}")

        return current_audio

    def _transcribe_audio(self, audio_np):
        """Transcribes audio using the Faster Whisper model."""
        self.get_logger().info("Transcribing audio, not recording")
        if audio_np is None:
            self.get_logger().warn("No audio data provided for transcription.")
            return ""
        
        """
        # Debug: Check audio data properties
        self.get_logger().info(f"Audio data shape: {audio_np.shape}, dtype: {audio_np.dtype}")
        self.get_logger().info(f"Audio data min: {np.min(audio_np):.4f}, max: {np.max(audio_np):.4f}, mean: {np.mean(audio_np):.4f}")
        """
        try:
            # Faster Whisper: Get segments and extract text
            segments, info = self.model.transcribe(audio_np, language='en', vad_filter=True)
            
            #self.get_logger().info(f"Transcription info: {info}")
            
            # Extract text from segments
            text = ""
            #segment_count = 0
            for segment in segments:
                #segment_count += 1
                #self.get_logger().info(f"Segment {segment_count}: {segment.text}")
                text += segment.text
            
            # self.get_logger().info(f"Total segments: {segment_count}")
            self.get_logger().info(f"Transcription result: '{text.strip()}'")
            return text.strip()

            # OLD METHOD (commented for revert):
            """
            # Whisper
            result = self.model.transcribe(audio_np, language='en', fp16=False) # fp16=False for CPU
            return result['text'].strip()
            """

        except Exception as e:
            self.get_logger().error(f"Error during transcription: {e}")
            return ""

            

    def _handle_command_timer(self):
        # Callback for the command mode timer.
        self.command_count += 1
        self.get_logger().info(f"Command timer incremented: {self.command_count}")
        if self.command_count >= 15:
            self.get_logger().info("Stopping command transcription after 15 seconds of inactivity.")
            if self.command_timer:
                self.command_timer.cancel()
                self.command_timer = None
            self.is_transcribing_commands = False

    def start_command_transcription(self):
        """Starts the command transcription mode."""
        self.get_logger().info("Entering command transcription mode. Say a command!")
        self.is_transcribing_commands = True
        # self.command_count = 0

        if not self.stream:
            self.get_logger().error("Failed to open audio stream for command mode. Cannot proceed.")
            self.is_transcribing_commands = False # Ensure loop doesn't start
            return

        if self.command_timer: # Ensure no old timer is running
            self.command_timer.cancel()
        # self.command_timer = self.create_timer(1.0, self._handle_command_timer)

        # rate = rospy.Rate(0.1)
        self.get_logger().info("Listening for command...\n Include direction, speed (1-5) and duration (in seconds)")
        time.sleep(5) # Give some time before starting to listen
        audio_np = self._get_audio_for_transcription((self.CHUNK_DURATION)*6)
        command = self._transcribe_audio(audio_np)
        
        # Create and publish the transcription message
        msg = String()
        msg.data = command
        self.transcription_pub.publish(msg) # Publish the raw transcription
        self.is_transcribing_commands = False
        time.sleep(20) # delay to allow ollama to transcribe the command before receiving new commands on whisper
        """
        while not rospy.is_shutdown() and self.is_transcribing_commands:
            rospy.loginfo("Waiting for command...")
            audio_np = self._get_audio_for_transcription((self.CHUNK_DURATION))
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
                    self.get_logger().info(f"Received valid command: {processed_command}")
                    msg = String()
                    msg.data = processed_command
                    self.transcription_pub.publish(msg)
                    self.command_count = 0 # Reset timer on valid command
                else:
                    self.get_logger().info(f"No recognized command in: '{command}'")
            else:
                self.get_logger().info("No audio data processed or transcription failed.")
            """
            
            #rate.sleep()

        # If loop exits (either shutdown or is_transcribing_commands becomes False)
        self.get_logger().info("Exited command transcription mode.")

    def wait_for_hello_loop(self):
        """Main loop to listen for 'hello'."""
        self.is_transcribing_commands = False # Ensure this is false

        # Ensure the stream is opened correctly *before* starting the main loop

        if not self.stream:
            self.get_logger().error("Failed to open audio stream for wake word. Node may not function.")
            # If we can't open stream, we can't listen. Keep trying or let node crash.
            # For now, let's keep trying in the loop, but it will error on process_audio_chunk.
            # A better solution here might be to spin ROS or wait, then retry.
            pass # Continue to the loop, where _process_audio_chunk will log errors

        # In ROS 2, we need to use time.sleep instead of rospy.Rate
        while not self._shutdown_requested and not self.is_transcribing_commands:
            self.get_logger().info("Waiting for 'hello'...")
            
            audio_np = self._get_audio_for_transcription(self.CHUNK_DURATION)
            transcription = self._transcribe_audio(audio_np)

            self.get_logger().info(f"Heard: '{transcription}'")

            if "hello" in transcription.lower(): # Using .lower() for robustness
                self.get_logger().info("Received hello!")
                self.start_command_transcription()
            time.sleep(1.0) # 1 Hz processing

        self.get_logger().info("Exited wake word listening loop.")
        self._close_audio_stream() # Ensure stream is closed upon loop exit


    def run(self):
        """Main entry point to start the node's operation."""
        # Initial state: waiting for "hello turtle"
        self.audio_thread.start()
        time.sleep(1)
        self.wait_for_hello_loop()

def main(args=None):
    rclpy.init(args=args)
    listener = None
    
    try:
        listener = WhisperListener()
        listener.run()  # This already starts the audio thread and runs the main loop
        
    except KeyboardInterrupt:
        if listener:
            listener.get_logger().info("Shutting down WhisperListener...")
    except Exception as e:
        if listener:
            listener.get_logger().error(f"Error in WhisperListener: {e}")
    finally:
        # Ensure proper cleanup sequence
        if listener:
            try:
                listener._on_shutdown()
                listener.destroy_node()
            except Exception as e:
                print(f"Error during listener cleanup: {e}")
        
        # Only shutdown if we initialized and ROS is still ok
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during ROS shutdown: {e}")

if __name__ == '__main__':
    main()