#!/usr/bin/env python
import rospy
import cv2
import provider
import threading
import time
import numpy as np
import mapmaker as mm
from sensor_msgs.msg import Image, NavSatFix
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from ros_gps_map.cfg import mapConfig

br = CvBridge()  
gpsMsg = None
zoom = 10
tileRadius = 1
hadFirstFix = False
mapmaker = None

def gpsCallback(msg):
    global gpsMsg, hadFirstFix, mapmaker
    gpsMsg = msg

    if hadFirstFix == False:
        hadFirstFix = True
        mapmaker = mm.MapMaker(msg.latitude, msg.longitude, zoom)
        mapmaker.start()

def dynamicCallback(config, level):
    global zoom, mapmaker
    print("zoom changed to lvl ", zoom)
    zoom = config["Zoom"]
    if mapmaker is not None:
        mapmaker.changeZoom(zoom)
    return config

def updateMap():
    global gpsMsg, mapmaker

    if gpsMsg is None or mapmaker is None:
        print("Waiting for gps message...")
        return

    lat = gpsMsg.latitude
    lon = gpsMsg.longitude

    mapmaker.mapImgLock.acquire()
    img = mapmaker.mapImg.copy()
    mapmaker.mapImgLock.release()

    dim = img.shape[0]
    img = cv2.circle(img, (dim//2, dim//2), 5, (0, 0, 255), -1, lineType=cv2.LINE_AA)

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
   
