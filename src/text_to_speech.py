#!/usr/bin/env python3

import festival
import soundfile as sf
import json
import rospy
from std_msgs.msg import String

class TextToSpeech:
    def __init__(self):
        rospy.init_node('text_to_speech')
        self.is_sim = rospy.get_param("~rivr", False)
        self.last_time_spoke = None
        self.rivr_robot_speech = rospy.Publisher('/robotspeech', String, queue_size=10)
        self.finger_sub = rospy.Subscriber('/text_to_speech', String, self.talk)

    def talk(self, str_msg):
        text = str_msg.data
        now = now = rospy.Time.now().to_sec()
        if (self.last_time_spoke is None or now > self.last_time_spoke+self.speech_delay):
            self.last_time_spoke = rospy.Time.now().to_sec()
            
            if self.is_sim:
                print("Saying rivr: " + text)
                wav = festival.textToWav(text)
                data = sf.read(wav)
                string_msg =json.dumps(list(data[0]))
                self.rivr_robot_speech.publish(string_msg)
            else:
                festival.sayText(str)

