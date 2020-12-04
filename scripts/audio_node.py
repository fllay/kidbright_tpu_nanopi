#!/usr/bin/env python
# license removed for brevity
import rospy


from sys import byteorder
from array import array
from struct import pack
import struct

import pyaudio
import wave
import numpy as np

#THRESHOLD = 500000
#THRESHOLD = 100000
THRESHOLD = 1000
CHUNK_SIZE = 2000
FORMAT = pyaudio.paInt16
RATE = 8000


from audio_common_msgs.msg import AudioData
from kidbright_tpu.msg import int1d
from std_msgs.msg import String


def is_silent(snd_data, thres):
    "Returns 'True' if below the 'silent' threshold"
    #print sum(np.multiply(snd_data, snd_data))/len(snd_data)
    return sum(np.multiply(snd_data, snd_data))/len(snd_data) < thres

def normalize(snd_data):
    "Average the volume out"
    MAXIMUM = 16384
    times = float(MAXIMUM)/max(abs(i) for i in snd_data)

    r = array('h')
    for i in snd_data:
        r.append(int(i*times))
    return r



def talker():
    #pub = rospy.Publisher('audio/audio', AudioData, queue_size=1)
    pub_a = rospy.Publisher('a1', String, queue_size=10)
    pub_aint = rospy.Publisher('audio_int', int1d, queue_size=10)
    

    rospy.init_node('audio_stream', anonymous=True)
    sampleRate = rospy.get_param('~samplingRate', 8000)
    nchannels = rospy.get_param('~nchannels', 1)
    soundCardNumber = rospy.get_param('~soundCardNumber', 2)
    thres = rospy.get_param('~THRESHOLD', 1000)
    print(sampleRate)    
    print(nchannels) 
    print(soundCardNumber) 
    print(thres)
    print '----------------------------'
    rate = rospy.Rate(10) # 10hz
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=nchannels, rate=sampleRate,
        input=True, output=False, input_device_index=soundCardNumber,
        frames_per_buffer=CHUNK_SIZE)

    num_silent = 0
    snd_started = False
    num_chunk = 0
    MAX_CHUNK = 12

    
    while not rospy.is_shutdown():
        snd_data = array('h', stream.read(CHUNK_SIZE))
        if byteorder == 'big':
            snd_data.byteswap()

        silent = is_silent(snd_data, thres)


        if silent and snd_started:
            pass
        elif not silent and not snd_started:
            snd_started = True
        if snd_started:
        #if True:
            num_chunk += 1
            #print len(snd_data)
            #print len(snd_data)
            #print "num chunk = %d" % num_chunk
            #print silent
            #ad = AudioData()
            #ad.data = tuple(np.fromstring(snd_data, dtype=np.uint8))
            #str1 =  bytes(snd_data) 
            #ad.data = snd_data.tolist()
            #pub.publish(ad)
            #rospy.loginfo(snd_data)
            bb = np.fromstring(snd_data, dtype=np.uint8).tobytes()
            cc = int1d()
            cc.data = np.fromstring(snd_data, dtype=np.int16)
            # print bb
            
            pub_a.publish(str(bb))
            pub_aint.publish(cc)
            
            if num_chunk >= MAX_CHUNK:
                num_chunk = 0
                snd_started = False
        else:
            pass
            

    stream.stop_stream()
    stream.close()
    p.terminate()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass