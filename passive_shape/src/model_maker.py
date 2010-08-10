#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("passive_shape")
import rospy
from numpy import *
import math
import cv
import os.path
from ShapeWindow import *
import ShapeWindowUtils
import Geometry2D
import pickle

class ModelMaker(ShapeWindow):
    
    def __init__(self,filepath,modelpath):
        bgd = cv.LoadImage(filepath,cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.modelpath = modelpath
        size = (bgd.width,bgd.height)
        ShapeWindow.__init__(self,name="Model Maker",size=size)
        self.shapesLock.acquire()
        self.img = cv.CloneImage(bgd)
        self.background = cv.CloneImage(bgd)
        self.shapesLock.release()
    
    def initExtended(self):
        self.lineColor = Colors.BLACK
        clearShapesButton = CVButton(text="CLEAR",bottomLeft=Geometry2D.Point(50,100), onClick=self.clearShapes)
        self.addOverlay(clearShapesButton)
        saveModelButton = CVButton(text="SAVE MODEL",bottomLeft=Geometry2D.Point(150,100), onClick = self.saveModel)
        self.addOverlay(saveModelButton)
        closeButton = CVButton(text="CLOSE",bottomLeft=Geometry2D.Point(350,100), onClick = self.close)
        self.addOverlay(closeButton)
    
    def onMouse(self,event,x,y,flags,param):
		return self.polyDrawer(event,x,y,flags,param)
		
    def saveModel(self):
        file = open(self.modelpath,'w')
        poly = self.getPolys()[0].getShape()
        pickle.dump(poly,file)
    
    

    
def main(args):
    corrected_filepath = args[0]
    corrected_modelpath = args[1]

    mm = ModelMaker(corrected_filepath,corrected_modelpath)
    while(not mm.isClosed()):
        rospy.sleep(0.05)
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
