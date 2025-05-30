#! /usr/bin/env/ python

import rospy
import pyaudio
import numpy as np
import whisper
from std_msgs.msg import String

# Whisper Model
model = whisper.load_model("tiny")

CHUNK_DURATION = 5
SAMPLE_RATE = 16000
CHUNK_SIZE = int(SAMPLE_RATE * CHUNK_DURATION)
FORMAT = pyaudio.paInt16
CHANNELS = 1
 
def transcribe():
    rospy.init_node("whisper_listener", anonymous=True)
    pub = rospy.Publisher('transcription_topic', String, queue_size=10)


def wait_for_hello():
    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT,
    channels=CHANNELS,

if __name__ == '__main__':
    wait_for_hello()    
