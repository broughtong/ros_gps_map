#!/usr/bin/env python
import rospy
import cv2
import provider
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge

br = CvBridge()  
gpsMsg = None

def gpsCallback(msg):
    global gpsMsg
    gpsMsg = msg


def updateMap():
    

    pub.publish(br.cv2_to_imgmsg(img))

if __name__ == '__main__':

    rospy.init_node("gps_map")

    pub = rospy.Publisher("gps_viz", Image, queue_size=1)
    rospy.Subscriber("/fix", NavSatFix, gpsCallback, queue_size=1)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        updateMap()
        r.sleep()
   
