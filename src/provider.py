#!/usr/bin/env python
import os
import math
import cv2
import threading
import urllib
from skimage import io

cacheFolder = "./mapCache"
urlTemplate = "http://tile.memomaps.de/tilegen/{z}/{x}/{y}.png"
urlFolder = urlTemplate.replace("/", "_")#.replace("\", "-")

def convertCoords(latitude, longitude, zoom):

    x = int((longitude + 180) / 360 * (2**zoom))
    y = int((1 - math.asinh(math.tan(math.radians(latitude))) / math.pi) / 2 * (2**zoom))
    
    return x, y, zoom

def isTileCoordinateValid(x, y, z):
    if x < 0 or y < 0 or z < 0 or z > 20:
        return False
    maxX = 2**z
    if x >= maxX:
        return False
    maxY = int((2**z) / 2)
    if y > maxY:
        return False
    return True

def getGPSCoordsPixelOffset(latitude, longitude, zoom):

    x = (longitude + 180) / 360 * (2**zoom)
    y = (1 - math.asinh(math.tan(math.radians(latitude))) / math.pi) / 2 * (2**zoom)

    imgCoords = convertCoords(latitude, longitude, zoom)
    x -= imgCoords[0]
    y -= imgCoords[1]

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

def getMapImage(tileX, tileY, tileZ):

    cachePath = os.path.join(cacheFolder, urlFolder)
    os.makedirs(cachePath, exist_ok=True)

    filename = os.path.join(cachePath, "%i-%i-%i.png" % (tileX, tileY, tileZ))

    image = None
    if os.path.isfile(filename):
        print("Cache hit for %s" % filename)
        image = cv2.imread(filename)
    else:
        if isTileCoordinateValid(tileX, tileY, tileZ) == False:
            return None
        image = downloadMapImage(tileX, tileY, tileZ)
        if image is None:
            return None
        cv2.imwrite(filename, image)
    return image

def cacheImages(lat, lon):

    threads = []
    for i in range(20):
        tileCoords = convertCoords(lat, lon, i)

        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0]-1, tileCoords[1]-1, tileCoords[2])))
        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0]-1, tileCoords[1], tileCoords[2])))
        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0]-1, tileCoords[1]+1, tileCoords[2])))
        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0], tileCoords[1]-1, tileCoords[2])))
        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0], tileCoords[1], tileCoords[2])))
        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0], tileCoords[1]+1, tileCoords[2])))
        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0]+1, tileCoords[1]-1, tileCoords[2])))
        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0]+1, tileCoords[1], tileCoords[2])))
        threads.append(threading.Thread(target=getMapImage, args=(tileCoords[0]+1, tileCoords[1]+1, tileCoords[2])))

    for t in threads:
        t.start()

if __name__ == '__main__':

    coords = (50.075538, 14.437800)
    for i in range(-1, 20):
        tileCoords = convertCoords(*coords, i)
        getMapImage(*tileCoords)

