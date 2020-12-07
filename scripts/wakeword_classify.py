#!/usr/bin/env python
import rospy
import wave
from std_msgs.msg import String
from kidbright_tpu.msg import float2d
import python_speech_features
import numpy as np
from kidbright_tpu.msg import int1d
# import tensorflow.compat.v1 as tf
# from tensorflow.keras import layers, models

sampleRate = 8000.0 # hertz



class save_wave():
    def __init__(self):

        rospy.init_node('wake_class_wait')
        
        #rospy.wait_for_message("audio/audio", AudioData)
        self.sampleRate = 8000
        # self.nFrame = 4
        self.frame_count = 0

        #self.number_subscriber = rospy.Subscriber("audio/audio", AudioData, self.callback, queue_size=1)
        self.a1_sub = rospy.Subscriber("/audio_int", int1d, self.callback, queue_size=1)
        
        
        rospy.loginfo("Record wave file")
       

        self.snd_data = []
        self.num_mfcc = 16
        self.len_mfcc = 16

        self.MFCCTextFileName = rospy.get_param('~mfcc_text_file', "foo_2.csv")
        self.nFrame = rospy.get_param('~nframe', 4)
        self.model_file = rospy.get_param('~model', "/home/pi/kb_2/models/model.h5")
        # self.model = models.load_model(self.model_file)
        
        

    def callback(self, msg):
       
        if self.frame_count < self.nFrame :    
            
            self.frame_count += 1
            #self.obj.writeframesraw(msg.data )
            #da_o = np.fromstring(msg.data, dtype=np.int16)
            #print(da_o)
            self.snd_data.extend(msg.data)

            print("here")
            print(self.frame_count)
            
            if self.frame_count == self.nFrame :
                pass
                
            
        else:
            #self.frame_count = 0
            print(type(self.snd_data[0]))
            print(len(self.snd_data))
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
            np.set_printoptions(suppress=True)
            print(type(mfccs))
            print(mfccs.shape)
            # print(mfccs)
            # print(np.matrix(mfccs))
            #np.savetxt('array_hf.csv', [mfccs], delimiter=',' , header='A Sample 2D Numpy Array :: Header', footer='This is footer')
            np.savetxt(self.MFCCTextFileName, mfccs, fmt='%f', delimiter=",")
            # np.savetxt("wav.csv", self.snd_data, fmt='%d', delimiter=",")
            print("Shuttting down")

            # # Reshape mfccs to have 1 more dimension
            # x = mfccs.reshape(mfccs.shape[0], 
            #               mfccs.shape[1], 
            #               1)

            # Make prediction
            # prediction = self.model.predict(x)
            # prediction = 1


            rospy.signal_shutdown("Term")
    
        # return 'prediction'




if __name__ == '__main__':
    print("hello")
    save_wave()
    try:
        rospy.spin()
    except:
        print("except")
        pass





