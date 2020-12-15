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
import pickle

sampleRate = 8000.0 # hertz


class inference():
    def __init__(self):

        rospy.init_node('wake_class_wait')

        # Settings
        self.sampleRate = 8000
        self.fps = int(self.sampleRate / 4)
        self.frame_count = 0
        self.snd_data = []
        self.num_mfcc = 16
        self.len_mfcc = 16
        self.start_index = 0
        self.window_stride = 2
        self.count = 0

        # Get params
        self.model_file = rospy.get_param('~model', "/home/pi/kb_2/models/model.h5")
        self.nFrame = rospy.get_param('~nframe', 4)
        self.output_type = rospy.get_param('~output_type', 'categorical_crossentropy')
        # self.binary_threshold = float(rospy.get_param('~binary_threshold', 0.5))
        self.categorical_threshold = float(rospy.get_param('~categorical_threshold', 0.5))
        self.terminate = bool(rospy.get_param('~terminate', False))
        # print('self.terminate\n', self.terminate, type(self.terminate))
        print('output_type:\n', self.output_type)

        # Load label_map
        with open('/home/pi/kb_2/label_map.pkl', 'rb') as pkl_file:
            self.label_map = pickle.load(pkl_file)

        # See if it is to be terminated
        if self.terminate:
            rospy.signal_shutdown("Term")
        
        # Subscribe to audio_int
        self.a1_sub = rospy.Subscriber("/audio_int", int1d, self.callback, queue_size=1)
        rospy.loginfo("Running inference ...")

        # Add publisher to publish inference result in real-time
        if self.output_type == 'binary_crossentropy':
            self.pred_pub = rospy.Publisher('inference', float1d, queue_size=10)
        else: # categorical_crossentropy
            self.pred_pub = rospy.Publisher('inference', String, queue_size=10)
        
       
        # Load the trained model
        self.model = models.load_model(self.model_file)

    def callback(self, msg):

        # Extend subscribed message            
        self.frame_count += 1
        self.snd_data.extend(msg.data)
                
        # Calculate MFCC and run inference every "window_stride" frames
        if self.frame_count == self.nFrame:
            if self.count == self.window_stride:
                # Calculate MFCC
                mfccs = python_speech_features.base.mfcc(np.array(self.snd_data[self.start_index*self.fps:(self.start_index+self.nFrame)*self.fps]), 
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

                # Reshape mfccs to have 1 more dimension
                x = mfccs.reshape(1, mfccs.shape[0], 
                                mfccs.shape[1], 1)

                # Make prediction
                prediction = self.model.predict(x)
                print(self.frame_count, ':', prediction)

                # Reset count
                self.count = 0

                # Add stride to start_index
                self.start_index += self.window_stride

                # Publish the prediction
                if self.output_type == 'binary_crossentropy':
                    self.pred_pub.publish(prediction)
                else: # categorical_crossentropy
                    # self.pred_pub.publish(self.label_map[np.argmax(prediction[0])])
                    # print(self.label_map[np.argmax(prediction[0])])

                    argmax = np.argmax(prediction[0])
                    if prediction[0][argmax] >= self.categorical_threshold:
                        self.pred_pub.publish(self.label_map[np.argmax(prediction[0])])
                        print(self.label_map[np.argmax(prediction[0])])
                    else:
                        self.pred_pub.publish('None')

        if self.frame_count > self.nFrame:# and self.frame_count % self.nFrame == 0:
            self.count += 1

            if self.count == self.window_stride:
                # Calculate MFCC
                mfccs = python_speech_features.base.mfcc(np.array(self.snd_data[self.start_index*self.fps:(self.start_index+self.nFrame)*self.fps]), 
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

                # Reshape mfccs to have 1 more dimension
                x = mfccs.reshape(1, mfccs.shape[0], 
                                mfccs.shape[1], 1)

                # Make prediction
                prediction = self.model.predict(x)
                print(self.frame_count, ':', prediction)

                # Reset count
                self.count = 0

                # Add stride to start_index
                self.start_index += self.window_stride

                # Publish the prediction
                if self.output_type == 'binary_crossentropy':
                    self.pred_pub.publish(prediction)
                else: # categorical_crossentropy
                    argmax = np.argmax(prediction[0])
                    if prediction[0][argmax] >= self.categorical_threshold:
                        self.pred_pub.publish(self.label_map[np.argmax(prediction[0])])
                        print(self.label_map[np.argmax(prediction[0])])
                    else:
                        self.pred_pub.publish('None')

                # # Publish the prediction
                # self.pred_pub.publish(prediction)


if __name__ == '__main__':
    print("hello meme")
    inference()
    try:
        rospy.spin()
    except:
        print("except")
        pass





