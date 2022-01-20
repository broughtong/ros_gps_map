#!/usr/bin/env python
import rospy
import cv2
import provider
import numpy as np
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
    img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

    dim = img.shape[0]

    baseImg = np.zeros((dim, dim, 3), np.uint8) 
    
    location = list(provider.getImageCoords(lat, lon, zoom))
    location[0] = int(location[0]*dim)
    location[1] = int(location[1]*dim)

    centre = (dim//2, dim//2)
    diff = (centre[0] - location[0], centre[1] - location[1])

    offsetX = diff[0]
    offsetY = diff[1]

    drawLocX = offsetX
    drawLocY = offsetY
    if offsetX > 0:
        img = img[:, :-offsetX]
    if offsetY > 0:
        img = img[:-offsetY, :]
    if offsetX < 0:
        img = img[:, -offsetX:]
        drawLocX = 0
    if offsetY < 0:
        img = img[-offsetY:, :]
        drawLocY = 0
    baseImg[drawLocY:drawLocY+img.shape[0], drawLocX:drawLocX+img.shape[1]] = img

    img = cv2.circle(baseImg, centre, 5, (0, 0, 255), -1, lineType=cv2.LINE_AA)
    #img = cv2.circle(img, (location[0], location[1]), 5, (0, 0, 255), -1, lineType=cv2.LINE_AA)

    img = br.cv2_to_imgmsg(img, encoding="passthrough")
    pub.publish(img)

if __name__ == '__main__':

    rospy.init_node("gps_map")

    srv = Server(mapConfig, dynamicCallback)

    pub = rospy.Publisher("gps_viz", Image, queue_size=1)
    rospy.Subscriber("/mobilicom_gps", NavSatFix, gpsCallback, queue_size=1)

    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        updateMap()
        r.sleep()
   
