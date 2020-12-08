#!/usr/bin/env python3.7
import rospy
import wave
from std_msgs.msg import String
from kidbright_tpu.msg import float2d, float1d
import python_speech_features
import numpy as np
from kidbright_tpu.msg import int1d
import tensorflow.compat.v1 as tf
from tensorflow.keras import layers, models
import time

sampleRate = 8000.0 # hertz


class inference():
    def __init__(self):

        rospy.init_node('wake_class_wait')
        
        # Subscribe to audio_int
        self.a1_sub = rospy.Subscriber("/audio_int", int1d, self.callback, queue_size=1)
        rospy.loginfo("Running inference ...")

        # Add publisher to publish inference result in real-time
        self.pred_pub = rospy.Publisher('inference', float1d, queue_size=10)
       
        # Settings
        self.sampleRate = 8000
        self.frame_count = 0
        self.snd_data = []
        self.num_mfcc = 16
        self.len_mfcc = 16

        # Get params
        self.model_file = rospy.get_param('~model', "/home/pi/kb_2/models/model.h5")

        # Load the trained model
        self.model = models.load_model(self.model_file)

    def callback(self, msg):

        # Extend subscribed message            
        self.frame_count += 1
        self.snd_data.extend(msg.data)
                
        # Calculate MFCC and run inference every 4 frames
        if self.frame_count > 0 and self.frame_count % 4 == 0:

            # Calculate MFCC
            mfccs = python_speech_features.base.mfcc(np.array(self.snd_data), 
                                        samplerate=self.sampleRate,
                                        winlen=0.256,
                                        winstep=0.050,
                                        numcep=self.num_mfcc,
                                        nfilt=26,
                                        nfft=2048,
                                        preemph=0.0,
                                        ceplifter=0,
                                        appendEnergy=False,
                                        winfunc=np.hanning)

            # Transpose MFCC, so that it is a time domain graph
            mfccs = mfccs.transpose()
            np.set_printoptions(suppress=True)
            # print(mfccs.shape)

            # Reshape mfccs to have 1 more dimension
            x = mfccs.reshape(1, mfccs.shape[0], 
                              mfccs.shape[1], 1)

            # Make prediction
            prediction = self.model.predict(x)
            print(self.frame_count, ':', prediction)
            # print('\nPrediction:\n', prediction)

            # Reset snd_data
            self.snd_data = []

            # Publish the prediction
            self.pred_pub.publish(prediction)

            # rospy.signal_shutdown("Term")
            

if __name__ == '__main__':
    print("hello")
    inference()
    try:
        rospy.spin()
    except:
        print("except")
        pass





