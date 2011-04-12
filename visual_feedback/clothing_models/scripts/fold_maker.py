#!/usr/bin/env python

##    @package snapshotter
#    This module provides basic functionality for taking a 'snapshot' of an image, and either pulling it for OpenCV
#   information, or saving it

import roslib
import sys
roslib.load_manifest("clothing_models")
import rospy
from numpy import *
import math
import cv
import os.path
from ShapeWindow import *
import ShapeWindowUtils
import Geometry2D
import pickle
import clothing_models.Vector2D
import Models
import time




class FoldMaker(ShapeWindow):
    
    def __init__(self,filepath,input_modelpath,output_modelpath):
        bgd = cv.LoadImage(filepath,cv.CV_LOAD_IMAGE_GRAYSCALE)
        self.input_modelpath = input_modelpath
        self.output_modelpath = output_modelpath
        self.initial_model = pickle.load(open(input_modelpath))
        self.initial_model.set_image(bgd)
        size = (bgd.width,bgd.height)
        ShapeWindow.__init__(self,name="Model Maker",size=size)
        self.shapesLock.acquire()
        self.img = cv.CloneImage(bgd)
        self.background = cv.CloneImage(bgd)
        self.initial_model.draw_to_image(self.background,cv.CV_RGB(255,0,0))
        self.shapesLock.release()
    
    def initExtended(self):
        self.lineColor = Colors.BLACK
        self.foldline_pts = []
        self.foldline = None
        self.has_foldline = False
        clearShapesButton = CVButton(text="CLEAR",bottomLeft=Geometry2D.Point(50,100), onClick=self.clearAll)
        self.addOverlay(clearShapesButton)
        saveModelButton = CVButton(text="SAVE MODEL",bottomLeft=Geometry2D.Point(150,100), onClick = self.saveModel)
        self.addOverlay(saveModelButton)
        closeButton = CVButton(text="CLOSE",bottomLeft=Geometry2D.Point(350,100), onClick = self.close)
        self.addOverlay(closeButton)
        
    def clearAll(self):
        self.clearShapes()
    
    def onMouse(self,event,x,y,flags,param):
        if not self.has_foldline:
            return self.foldDrawer(event,x,y,flags,param)
            
    def foldDrawer(self,event,x,y,flags,param):
        if event==cv.CV_EVENT_LBUTTONUP:
            self.foldline_pts.append((x,y))
            print "ADDED PT"
            cv.Circle(self.background,(x,y),3,cv.CV_RGB(255,0,0),-1)
            if len(self.foldline_pts) >= 2:
                self.foldline = Vector2D.make_ln_from_pts(self.foldline_pts[0],self.foldline_pts[1])
                ln_start = Vector2D.intercept(self.foldline,Vector2D.horiz_ln(y=0))
                ln_end = Vector2D.intercept(self.foldline,Vector2D.horiz_ln(y=self.background.height))
                cv.Line(self.background,ln_start,ln_end,cv.CV_RGB(0,0,0))
                self.has_foldline = True
        elif len(self.foldline_pts) > 0:
            self.addTempCVShape(CVLineSegment(cv.CV_RGB(255,255,255),2,Geometry2D.LineSegment(Geometry2D.Point(self.foldline_pts[0][0],self.foldline_pts[0][1]),Geometry2D.Point(x,y))))
    
    def permanentHighlightPt(self,pt):
        self.addCVShape(CVCircle(cv.CV_RGB(0,0,0),self.front(),Geometry2D.Circle(pt,3)))    
    
    def permanentHighlightSegment(self,seg):
        self.addCVShape(CVLineSegment(cv.CV_RGB(0,0,0),self.front(),seg))  
            
           
    
    def saveModel(self):
        file = open(self.output_modelpath,'w')
        displacement = Vector2D.intercept(self.foldline,Vector2D.horiz_ln(y=0))[0]
        (dx,dy) = Vector2D.line_vector(self.foldline)
        angle = abs(arctan(dy / dx))
        if dx > 0 and dy > 0:
            angle = angle
        elif dx < 0 and dy > 0:
            angle = pi - angle
        elif dx < 0 and dy < 0:
            angle = pi + angle
        else:
            angle *= -1
        model = Models.Point_Model_Folded(self.initial_model,self.foldline_pts[0],self.foldline_pts[1])
        model.draw_to_image(self.background,cv.RGB(255,0,0))
        if model.illegal() or model.structural_penalty() >= 1.0:
            print "Model is illegal!"
            self.clearAll()
        else:
            model.set_image(None)
            pickle.dump(model,file)
        

    
def main(args):
    filepath = args[0]
    input_modelpath = args[1]
    output_modelpath = args[2]

    fm = FoldMaker(filepath,input_modelpath,output_modelpath)
    while(not fm.isClosed()):
        rospy.sleep(0.05)
    return
    
if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
