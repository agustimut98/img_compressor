#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def talker():
    pub = rospy.Publisher('original_image', Image, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1/60) # 60seconds
    bridge = CvBridge()
    img = cv2.imread('/home/agusti/Descargas/TFM/ICVV-2017_DEBT/lena.pgm',1)
    msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
    
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing image")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
