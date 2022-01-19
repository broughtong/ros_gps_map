#!/usr/bin/env python
import rospy
import cv2
import provider
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from ros_gps_map.cfg import mapConfig

br = CvBridge()  
gpsMsg = None
zoom = 0

def gpsCallback(msg):
    global gpsMsg
    gpsMsg = msg

def dynamicCallback(config, level):
    global zoom
    zoom = config["Zoom"]
    return config

def updateMap():
    global gpsMsg

    if gpsMsg is None:
        print("Waiting for gps message...")
        return

    lat = gpsMsg.latitude
    lon = gpsMsg.longitude

    img = provider.getMapImage(lat, lon, zoom)
    if img is None:
        return

    dim = img.shape[0]

    location = list(provider.getImageCoords(lat, lon, zoom))
    location[0] = int(location[0]*dim)
    location[1] = int(location[1]*dim)

    img = cv2.circle(img, (location[0], location[1]), 5, (0, 0, 255), -1, lineType=cv2.LINE_AA)

    img = br.cv2_to_imgmsg(img, encoding="passthrough")
    pub.publish(img)

if __name__ == '__main__':

    rospy.init_node("gps_map")

    srv = Server(mapConfig, dynamicCallback)

    pub = rospy.Publisher("gps_viz", Image, queue_size=1)
    rospy.Subscriber("/mobilicom_gps", NavSatFix, gpsCallback, queue_size=1)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        updateMap()
        r.sleep()
   
