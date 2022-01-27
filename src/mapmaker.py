#!/usr/bin/env python
import rospy
import cv2
import provider
import threading
import time
import numpy as np
from sensor_msgs.msg import NavSatFix

class MapMaker(threading.Thread):
    def __init__(self, lat, lon, zoom):
        threading.Thread.__init__(self)

        self.zoom = zoom

        provider.cacheImages(lat, lon)

        self.cacheRadius = 1
        self.cachedMaps = []
        self.cacheLock = threading.Lock()
        cacheDimension = (2*self.cacheRadius) + 1
        for i in range(cacheDimension):
            self.cachedMaps.append([])
            for j in range(cacheDimension):
                self.cachedMaps[i].append(None)

        self.lat, self.lon = lat, lon
        self.cacheCentre = provider.convertCoords(lat, lon, self.zoom)
        sample = provider.getMapImage(*self.cacheCentre) 
        self.imgDim = sample.shape[0]

        self.mapImgLock = threading.Lock()
        self.mapImg = np.zeros((self.imgDim, self.imgDim, 3), np.uint8)

    def changeZoom(self, zoom):
        self.zoom = zoom
        self.cachedMaps = []
        cacheDimension = (2*self.cacheRadius) + 1
        for i in range(cacheDimension):
            self.cachedMaps.append([])
            for j in range(cacheDimension):
                self.cachedMaps[i].append(None)
        self.cacheCentre = provider.convertCoords(self.lat, self.lon, self.zoom)
        self.redraw()

    def changePosition(self, lat, lon):

        pass

    def getImage(self, tileX, tileY, cacheX, cacheY):
        img = provider.getMapImage(tileX, tileY, self.zoom)
        if img is None:
            print("Unable to cache!", tileX, tileY, cacheX, cacheY, self.zoom)
            return
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        self.cacheLock.acquire()
        self.cachedMaps[cacheX][cacheY] = img
        self.cacheLock.release()

    def run(self):

        while not rospy.is_shutdown():

            if self.cacheCentre is None:
                time.sleep(0.5)
                print("No info, waiting")
                continue

            threads = []
            cacheDimension = (2*self.cacheRadius) + 1
            for i in range(cacheDimension):
                for j in range(cacheDimension):
                    if self.cachedMaps[i][j] is None:
                        tileX = self.cacheCentre[0] + i - int((cacheDimension - 1) / 2)
                        tileY = self.cacheCentre[1] + j - int((cacheDimension - 1) / 2)
                        t = threading.Thread(target=self.getImage, args=(tileX, tileY, i, j))
                        t.start()
                        threads.append(t)
            for t in threads:
                t.join()

            #for i in range(cacheDimension):
            #    for j in range(cacheDimension):
            #        cv2.imwrite("%i-%i.png" % (i, j), self.cachedMaps[i][j])

            self.redraw()
            print("redrew")
            time.sleep(0.5)

            #print(i*self.imgDim+j, i, j, i - int((self.cacheRadius - 1) / 2), j - int((n-1)/2))

    def redraw(self):
    
        self.mapImgLock.acquire()
        self.mapImg = np.zeros((self.imgDim, self.imgDim, 3), np.uint8)

        offsets = list(provider.getGPSCoordsPixelOffset(self.lat, self.lon, self.zoom))
        cacheDimension = (2*self.cacheRadius) + 1
        for i in range(cacheDimension):
            for j in range(cacheDimension):
                centre = int((len(self.cachedMaps) - 1) / 2)
                tileX = i - centre
                tileY = j - centre

                x = tileX
                y = tileY
                x += (0.5 - offsets[0])
                y += (0.5 - offsets[1])
                self.blit(self.mapImg, x, y, self.cachedMaps[i][j])

        print("amere")
        cv2.imwrite("nsnsn.png", self.mapImg)
        self.mapImgLock.release()

        #self.mapImg = cv2.circle(self.mapImg, (self.imgDim//2, self.imgDim//2), 5, (0, 0, 255), -1, lineType=cv2.LINE_AA)

    def blit(self, baseImg, x, y, img):

        if img is None:
            img = np.zeros((self.imgDim, self.imgDim, 3), np.uint8)
            img[:] = 128

        x = int(x * self.imgDim)
        y = int(y * self.imgDim)

        drawLocX = x
        drawLocY = y
        if x > 0:
            img = img[:, :-x]
        if y > 0:
            img = img[:-y, :]
        if x < 0:
            img = img[:, -x:]
            drawLocX = 0
        if y < 0:
            img = img[-y:, :]
            drawLocY = 0
        baseImg[drawLocY:drawLocY+img.shape[0], drawLocX:drawLocX+img.shape[1]] = img

if __name__ == '__main__':

    zoom = 18
    mapMaker = MapMaker(50.075538, 14.437800, zoom)
    mapMaker.start()
