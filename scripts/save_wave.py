#!/usr/bin/env python
import rospy
import wave
from std_msgs.msg import String

sampleRate = 8000.0 # hertz

class save_wave():
    def __init__(self):
      
        rospy.init_node("wave_wait")
        #rospy.wait_for_message("audio/audio", AudioData)

        # Set parameters - wav
        self.sampleRate = rospy.get_param('~samplingRate', 8000)
        self.fileName = rospy.get_param('~file', "sound.wav")
        self.nFrame = rospy.get_param('~nframe', 5)
        print self.fileName 
        self.frame_count = 0

        # Set parameters - MFCC
        self.snd_data = []
        self.num_mfcc = 16
        self.len_mfcc = 16

        #self.number_subscriber = rospy.Subscriber("audio/audio", AudioData, self.callback, queue_size=1)
        # Subscribe to "a1" publisher
        self.a1_sub = rospy.Subscriber("a1", String, self.callback, queue_size=1)
        
        # Set wav object
        rospy.loginfo("Record wave file")
        self.obj = wave.open(self.fileName,'w')
        self.obj.setnchannels(1) # mono
        self.obj.setsampwidth(2)
        self.obj.setframerate(self.sampleRate)

        # Set MFCC object
        # self.mfcc_obj = np.savefig???


    def callback(self, msg):
           
        if self.frame_count < self.nFrame :    
            
            # Write msg from publisher to wav
            self.frame_count += 1
            self.obj.writeframesraw(msg.data )

            # Append msg from publisher to list
            da_o = np.fromstring(msg.data, dtype=np.int16)
            print da_o
            self.snd_data.extend(da_o)

            # Print log
            print "here"
            print self.frame_count

            # Close wav object
            if self.frame_count == self.nFrame :
                self.obj.close()
            
            print 'Wav file saved successfully.'
                  
        else: # once recording is done
            #self.frame_count = 0

            # Convert snd_data to MFCC and save it
            print type(self.snd_data[0])
            print len(self.snd_data)
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
            print type(mfccs)
            print mfccs.shape
            #print(mfccs)
            print np.matrix(mfccs)
            #np.savetxt('array_hf.csv', [mfccs], delimiter=',' , header='A Sample 2D Numpy Array :: Header', footer='This is footer')
            np.savetxt("foo.csv", mfccs, fmt='%f', delimiter=",")
            print 'MFCC saved successfully.'

            # Shutdown node
            print "Shuttting down"
            rospy.signal_shutdown("Term")
    
# class save_mfcc():
#     def __init__(self):

#         rospy.init_node("mfcc_wait")
#         #rospy.wait_for_message("audio/audio", AudioData)
#         self.sampleRate = rospy.get_param('~samplingRate', 8000)
#         self.fileName = rospy.get_param('~file', "sound.wav")
#         self.nFrame = rospy.get_param('~nframe', 5)
#         print self.fileName 
#         self.frame_count = 0

#         #self.number_subscriber = rospy.Subscriber("audio/audio", AudioData, self.callback, queue_size=1)
#         self.a1_sub = rospy.Subscriber("a1", String, self.callback, queue_size=1)
        
#         rospy.loginfo("Record wave file")
#         self.obj = wave.open(self.fileName,'w')
#         self.obj.setnchannels(1) # mono
#         self.obj.setsampwidth(2)
#         self.obj.setframerate(self.sampleRate)


#         mfccs = python_speech_features.base.mfcc(np.array(self.snd_data), 
#                                         samplerate=self.sampleRate,
#                                         winlen=0.256,
#                                         winstep=0.050,
#                                         numcep=self.num_mfcc,
#                                         nfilt=26,
#                                         nfft=2048,
#                                         preemph=0.0,
#                                         ceplifter=0,
#                                         appendEnergy=False,
#                                         winfunc=np.hanning)

if __name__ == '__main__':
    print "hello"
    try:
        save_wave()
        
        rospy.spin()
    except:
        print "except"
        pass




