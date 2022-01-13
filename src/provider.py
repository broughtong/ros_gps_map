#!/usr/bin/env python
import os
import math
import cv2
import threading
import urllib
from skimage import io

cacheFolder = "./mapCache"
urlTemplate = "https://tiles.wmflabs.org/osm-no-labels/{z}/{x}/{y}.png"
urlFolder = urlTemplate.replace("/", "_")#.replace("\", "-")

def convertCoords(latitude, longitude, zoom):

    x = int((longitude + 180) / 360 * (2**zoom))
    y = int((1 - math.asinh(math.tan(math.radians(latitude))) / math.pi) / 2 * (2**zoom))
    
    return x, y, zoom

def downloadMapImage(x, y, z):

    url = urlTemplate.format(x=x, y=y, z=z)
    print("Acquiring map image %s" % (url))

    try:
        image = io.imread(url)
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)
    except urllib.error.HTTPError as e:
        print("Error accessing map data! %s %s" % (e, url))
        return None

    return image

def getMapImage(lat, lon, z):

    cachePath = os.path.join(cacheFolder, urlFolder)
    if not os.path.exists(cacheFolder):
        os.mkdir(cacheFolder)
    if not os.path.exists(cachePath):
        os.mkdir(cachePath)

    mapCoords = convertCoords(lat, lon, z)
    filename = os.path.join(cachePath, "%i-%i-%i.png" % mapCoords)

    image = None
    if os.path.isfile(filename):
        print("Cache hit for %s" % filename)
        image = cv2.imread(filename)
    else:
        image = downloadMapImage(*mapCoords)
        if image is None:
            return None
        cv2.imwrite(filename, image)
    return image

if __name__ == '__main__':

    coords = (50.075538, 14.437800)
    for i in range(-1, 20):
        getMapImage(*coords, i)
        lcoords = convertCoords(*coords, i)
        image = downloadMapImage(lcoords[0], *lcoords[1:])

