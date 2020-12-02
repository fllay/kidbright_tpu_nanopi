#!/usr/bin/env python
import rospy
import wave
from std_msgs.msg import String


sampleRate = 8000.0 # hertz



class save_wave():
    def __init__(self):
      
        rospy.init_node("wave_wait")
        #rospy.wait_for_message("audio/audio", AudioData)
        self.sampleRate = rospy.get_param('~samplingRate', 8000)
        self.fileName = rospy.get_param('~file', "sound.wav")
        self.nFrame = rospy.get_param('~nframe', 5)
        print self.fileName 
        self.frame_count = 0

        #self.number_subscriber = rospy.Subscriber("audio/audio", AudioData, self.callback, queue_size=1)
        self.a1_sub = rospy.Subscriber("a1", String, self.callback, queue_size=1)
        
        
        rospy.loginfo("Record wave file")
        self.obj = wave.open(self.fileName,'w')
        self.obj.setnchannels(1) # mono
        self.obj.setsampwidth(2)
        self.obj.setframerate(self.sampleRate)

       

    def callback(self, msg):
       
            
        if self.frame_count < self.nFrame :    
            
            self.frame_count += 1
            self.obj.writeframesraw(msg.data )

            print("here")
            print(self.frame_count)
            if self.frame_count == self.nFrame :
                self.obj.close()
                
            
        else:
            #self.frame_count = 0
            print "Shuttting down"
            rospy.signal_shutdown("Term")
    




if __name__ == '__main__':
    print "hello"
    try:
        save_wave()
        
        rospy.spin()
    except:
        print "except"
        pass




