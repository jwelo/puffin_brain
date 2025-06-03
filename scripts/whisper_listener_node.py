#! /usr/bin/env python

import rospy
import pyaudio
import numpy as np
import whisper
from std_msgs.msg import String

CHUNK_DURATION = 5
SAMPLE_RATE = 16000
CHUNK_SIZE = int(SAMPLE_RATE * CHUNK_DURATION)
FORMAT = pyaudio.paInt16
CHANNELS = 1
global count
global p
global stream
# Whisper Model
model = whisper.load_model("tiny")

def timer_increment(event):
    global count
    count += 1
    rospy.loginfo(f"Timer incremented: {count}")
    if count >= 30:
        rospy.loginfo("Stopping transcription after 30 seconds")
        # stop recording and close the stream
        stream.stop_stream()
        stream.close()
        p.terminate()
        wait_for_hello()

def transcribe():
    pub = rospy.Publisher('transcription_topic', String, queue_size=10)
    global count
    count = 0
    rospy.Timer(rospy.Duration(1), timer_increment)
    p = pyaudio.PyAudio()
    try:
        stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    input=True,
                    frames_per_buffer=CHUNK_SIZE)
    except Exception as e:
        rospy.logerr(f"Error initializing audio stream or loading model: {e}")
        return
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        rospy.loginfo("waiting for command")
        audio_data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
        audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
        audio_np = whisper.pad_or_trim(audio_np, whisper.audio.N_SAMPLES)
        
        # Transcribe the audio
        command_result = model.transcribe(audio_np, language='en')
        command = command_result['text'].strip()

        msg = String()
        if "left" in command:
            msg.data = "left"
            rospy.loginfo("received left command")
            count = 0
        elif "right" in command:
            msg.data = "right"
            rospy.loginfo("received right command")
            count = 0
        elif "forward" in command:
            msg.data = "forward"
            rospy.loginfo("received forward command")
            count = 0
        elif "backward" in command:
            msg.data = "backward"
            rospy.loginfo("received backward command")
            count = 0
        elif "stop" in command:
            msg.data = "stop"
            rospy.loginfo("received stop command")
            count = 0
        rospy.loginfo("Recevied command: %s", msg)
        pub.publish(msg)
        rate.sleep()


def wait_for_hello():
    p = pyaudio.PyAudio()
    try:
        stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=SAMPLE_RATE,
                    input=True,
                    frames_per_buffer=CHUNK_SIZE)
                    
    except Exception as e:
        rospy.logerr(f"Error initializing audio stream or loading model: {e}")
        return
    
    rospy.loginfo("Listening for audio...")
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        rospy.loginfo("waiting for hello")
        audio_data = stream.read(CHUNK_SIZE, exception_on_overflow=False)
        audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
        audio_np = whisper.pad_or_trim(audio_np, whisper.audio.N_SAMPLES)
        
        # Transcribe the audio
        result = model.transcribe(audio_np, language='en')
        transcription = result['text'].strip()


        if transcription == "hello turtle":
            rospy.loginfo("received hello")
            stream.stop_stream()
            stream.close()
            p.terminate()
            transcribe()
        rate.sleep()
    
    # stop recording and close the stream
    stream.stop_stream()
    stream.close()
    p.terminate()
    rospy.loginfo("Stopped recording and closed the stream.")
                

if __name__ == '__main__':
    rospy.init_node("whisper_listener", anonymous=True)
    wait_for_hello()    
