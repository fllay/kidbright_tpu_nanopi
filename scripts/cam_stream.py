#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

def camThread(cam):


    #cam = cv2.VideoCapture('rkisp device=/dev/video6 io-mode=4 path-iqf=/etc/cam_iq/ov13850.xml ! video/x-raw,format=NV12,width=640,height=480,framerate=15/1 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
    
    image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)
    rospy.init_node('cam_stream', anonymous=False)
    rate = rospy.Rate(15) # 10hz

    while not rospy.is_shutdown():
     
        ret, color_image = cam.read()
        if not ret:
            print("no image")
            continue
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', color_image)[1]).tostring()
        # Publish new image
        image_pub.publish(msg)
        
        rate.sleep()

    cam.release() 


if __name__ == '__main__':
    #cam = cv2.VideoCapture(1)
    #cam.set(5 , 30) 
    cam = cv2.VideoCapture('rkisp device=/dev/video1 io-mode=4 ! video/x-raw,format=NV12,width=640,height=480,framerate=15/1 ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

    try:
        camThread(cam)

    except rospy.ROSInterruptException:
        cam.release() 
        pass
